
#include <iostream>
#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <math.h>

using namespace std::chrono_literals;

static bool draw_points = false;
static bool draw_neighborhood = false;
static bool draw_voxels = false;
static bool draw_normals = false;

static double p = 0.2;
double get_delta() {
  return 2 * p;
}
static pcl::PointXYZ min_point_AABB;
static pcl::PointXYZ max_point_AABB;

/*------------------------ MESH COMPONENTS ------------------------*/

class Vertex {
public:
  bool used;
  pcl::PointXYZ pos;
  pcl::Normal normal;
  
  pcl::PointXYZ *voxelPos() {
    return new pcl::PointXYZ((pos.x - min_point_AABB.x) / get_delta(), (pos.y - min_point_AABB.y) / get_delta(), (pos.z - min_point_AABB.z) / get_delta());
  }
  std::string to_string() {
    std::ostringstream ss;
    ss << "Vertex(" << pos.x << ", " << pos.y << ", " << pos.z << ")";
    return ss.str();
  }
  std::vector<Vertex*> *edges;
};

static std::vector<std::vector<Vertex*>*> *boundary_edges;

class Triangle {
public:
  std::vector<Vertex*> points;
  void drawTriangle(pcl::visualization::PCLVisualizer *viewer);
  std::string to_string() {
    std::ostringstream ss;
    ss << "Triangle(" << points[0]->to_string() << ", " << points[1]->to_string() << ", " << points[2]->to_string() << ")";
    return ss.str();
  }
};

static std::vector<Triangle*> *triangles;

void Triangle::drawTriangle(pcl::visualization::PCLVisualizer *viewer) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr triangle (new pcl::PointCloud<pcl::PointXYZ>);
  triangle->push_back(points[0]->pos);
  triangle->push_back(points[1]->pos);
  triangle->push_back(points[2]->pos);
  viewer->addPolygon<pcl::PointXYZ>(triangle, 1.0, 1.0, 1.0, to_string());
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, to_string());
}

class Voxel {
public:
  pcl::PointXYZ origin;
  std::vector<Vertex*> *vertices;
  void drawRect(pcl::visualization::PCLVisualizer *viewer);

};

enum EdgeType { active, boundary, inner };

class Edge {
public:
  Edge(Vertex *vi, Vertex *vj, Vertex *vo, pcl::PointXYZ cent) {
    v_i = vi;
    v_j = vj;
    v_o = vo;
    c = cent;
    facets = new std::vector<Triangle*>();
  }
  Vertex *v_i;
  Vertex *v_j;
  Vertex *v_o;
  pcl::PointXYZ c;
  std::vector<Triangle*> *facets;
};

/*------------------------ VOXEL GRID ------------------------*/

class VoxelGrid {
public:
  int num_x;
  int num_y;
  int num_z;
  std::vector<Voxel*> voxels;
  std::vector<Voxel*> *findNeighboringVoxels(int x_p, int y_p, int z_p);
  Voxel *findVoxel(int x, int y, int z) {
    if (x < num_x && y < num_y && z < num_z && x >= 0 && y >= 0 && z >= 0) {
      return voxels.at(x * (num_y * num_z) + y * (num_z) + z);
    } else {
      return NULL;
    }
  }
  std::vector<Vertex*> *findNeighborhood(Vertex *v) {
    std::vector<Vertex*> *neighbors = new std::vector<Vertex*>();
    for (Voxel *vox : *findNeighboringVoxels(v->voxelPos()->x, v->voxelPos()->y, v->voxelPos()->z)) {
      for (Vertex *ver : *vox->vertices) {
        if (ver != v) {
          neighbors->push_back(ver);
        }
      }
    }
    return neighbors;
  }
};

std::vector<Voxel*> *VoxelGrid::findNeighboringVoxels(int x_p, int y_p, int z_p) {
  std::vector<Voxel*> *neighboring_voxels (new std::vector<Voxel*>);
  for (int x = x_p - 1; x <= x_p + 1; x++) {
    for (int y = y_p - 1; y <= y_p + 1; y++) {
      for (int z = z_p - 1; z <= z_p + 1; z++) {
        Voxel *v = findVoxel(x, y, z);
        if (v != NULL) {
          neighboring_voxels->push_back(v);
        }
      }
    }
  }
  return neighboring_voxels;
}

/*------------------------ FRONT & FRONT MAINTAINENCE ------------------------*/

class Front {
public:
  std::vector<Edge*> *front;
  bool onFront(Vertex *x);
  void join(Edge *e, Vertex *x, pcl::PointXYZ *cent);
  void glue(Edge *e);
};

static Front *front;

bool Front::onFront(Vertex *x) {
  for (Edge *e : *front) {
    if (e->v_i == x || e->v_j == x) {
      return true;
    }
  }
  return false;
}

void Front::join(Edge *ep, Vertex *k, pcl::PointXYZ *cent) {
  Edge *eik = new Edge(ep->v_i, k, ep->v_j, *cent);
  Edge *ekj = new Edge(k, ep->v_j, ep->v_i, *cent);
  front->push_back(eik);
  front->push_back(ekj);
  eik->v_i->edges->push_back(eik->v_j);
  ekj->v_i->edges->push_back(ekj->v_j);
}

bool is_used_edge(Edge *e) {
  bool is_used_1 = false;
  bool is_used_2 = false;
  for (Vertex *v : *e->v_i->edges) {
    if (v == e->v_j) {
      is_used_1 = true;
    }
  }
  for (Vertex *v : *e->v_j->edges) {
    if (v == e->v_i) {
      is_used_2 = true;
    }
  }
  return is_used_1 and is_used_2;
}

bool is_boundary(Edge *e) {
  for (std::vector<Vertex*> *v : *boundary_edges) {
    if ((e->v_i == v->at(0) && e->v_j == v->at(1)) ||
        (e->v_j == v->at(0) && e->v_i == v->at(1)))
    {
      return true;
    }
  }
  return false;
}

void marked_boundary(Edge *e) {
  boundary_edges->push_back(new std::vector<Vertex*>{e->v_i, e->v_j});
}

Edge *get_active_edge() {
  for (Edge *e : *front->front) {
    if (!is_used_edge(e) && !is_boundary(e)) {
      return e;
    }
  }
  return NULL;
}

/*------------------------ BALL PIVOTING ALGORITHM ------------------------*/

double calc_radius (Triangle *tri) {
  Eigen::Vector3d p1(tri->points.at(0)->pos.x, tri->points.at(0)->pos.y, tri->points.at(0)->pos.z);
  Eigen::Vector3d p2(tri->points.at(1)->pos.x, tri->points.at(1)->pos.y, tri->points.at(1)->pos.z);
  Eigen::Vector3d p3(tri->points.at(2)->pos.x, tri->points.at(2)->pos.y, tri->points.at(2)->pos.z);

  return (p1 - p2).norm() * (p2 - p3).norm() * (p3 - p1).norm() / (2 * (p1-p2).cross(p2-p3).norm());
}

pcl::PointXYZ calc_circ_center (Triangle *tri) {
  Eigen::Vector3d p1(tri->points.at(0)->pos.x, tri->points.at(0)->pos.y, tri->points.at(0)->pos.z);
  Eigen::Vector3d p2(tri->points.at(1)->pos.x, tri->points.at(1)->pos.y, tri->points.at(1)->pos.z);
  Eigen::Vector3d p3(tri->points.at(2)->pos.x, tri->points.at(2)->pos.y, tri->points.at(2)->pos.z);


  Eigen::Vector3d p1mp2 = p1 - p2;
  Eigen::Vector3d p2mp3 = p2 - p3;
  Eigen::Vector3d p1mp3 = p1 - p3;
  Eigen::Vector3d p2mp1 = p2 - p1;
  Eigen::Vector3d p3mp1 = p3 - p1;
  Eigen::Vector3d p3mp2 = p3 - p2;

  double denom = (2 * (p1mp2.cross(p2mp3)).norm() * (p1mp2.cross(p2mp3)).norm());

  double alpha = p2mp3.norm() * p2mp3.norm() * p1mp2.dot(p1mp3) / denom;

  double beta = p1mp3.norm() * p1mp3.norm() * p2mp1.dot(p2mp3) / denom;

  double gamma = p1mp2.norm() * p1mp2.norm() * p3mp1.dot(p3mp2) / denom;

  Eigen::Vector3d pc = (alpha * p1 + beta * p2 + gamma * p3);

  return pcl::PointXYZ(pc[0], pc[1], pc[2]);
}

pcl::PointXYZ get_outward_facing_normal (Triangle *tri) {
  Eigen::Vector3d p1(tri->points.at(0)->pos.x, tri->points.at(0)->pos.y, tri->points.at(0)->pos.z);
  Eigen::Vector3d p2(tri->points.at(1)->pos.x, tri->points.at(1)->pos.y, tri->points.at(1)->pos.z);
  Eigen::Vector3d p3(tri->points.at(2)->pos.x, tri->points.at(2)->pos.y, tri->points.at(2)->pos.z);

  Eigen::Vector3d n1(tri->points.at(0)->normal.normal_x, tri->points.at(0)->normal.normal_y, tri->points.at(0)->normal.normal_z);
  Eigen::Vector3d n2(tri->points.at(1)->normal.normal_x, tri->points.at(1)->normal.normal_y, tri->points.at(1)->normal.normal_z);
  Eigen::Vector3d n3(tri->points.at(2)->normal.normal_x, tri->points.at(2)->normal.normal_y, tri->points.at(2)->normal.normal_z);

  Eigen::Vector3d ntri = (p2 - p1).cross(p3 - p1);
  ntri.normalize();
  return pcl::PointXYZ(ntri[0], ntri[1], ntri[2]);

}

bool contains_no_other_data_points (pcl::PointXYZ pball_center, double pball_radius, Triangle *tri, VoxelGrid *vg) {
  Vertex *v = new Vertex();
  v->pos = pball_center;
  for (Vertex *neighbor : *vg->findNeighborhood(v)) {
    if (neighbor != tri->points[0] && neighbor != tri->points[1] && neighbor != tri->points[2]) {
      if (abs(pcl::euclideanDistance(pball_center, neighbor->pos)) <= pball_radius) {
        return false;
      }
    }
  }
  return true;
}


Triangle *seed_triangle(std::vector<Vertex*> *vertices, VoxelGrid *vg, pcl::visualization::PCLVisualizer *viewer, pcl::PointXYZ *seedcent) {
  for (Vertex *v : *vertices)
  {
    if (!v->used) {
      std::vector<Vertex*> *neighborhood = vg->findNeighborhood(v);
      std::sort(neighborhood->begin(), neighborhood->end(),
                [v](const Vertex* lhs, const Vertex* rhs){
                  return pcl::euclideanDistance(v->pos, lhs->pos) < pcl::euclideanDistance(v->pos, rhs->pos);
                });
      for (int a = 0; a < neighborhood->size(); a++) {
        for (int b = a + 1; b < neighborhood->size(); b++) {
          Triangle *tri = new Triangle();
          tri->points = std::vector<Vertex*>{v, neighborhood->at(a), neighborhood->at(b)};
            pcl::PointXYZ tri_normal = get_outward_facing_normal(tri);
            double radius = calc_radius (tri);
            double normal_dist = p * p - radius * radius;
            if (normal_dist > 0) {
              pcl::PointXYZ c = calc_circ_center (tri);
              pcl::PointXYZ pball_center(c.x + sqrt(abs(normal_dist)) * tri_normal.x, c.y + sqrt(abs(normal_dist)) * tri_normal.y, c.z + sqrt(abs(normal_dist)) * tri_normal.z);
              if (contains_no_other_data_points(pball_center, radius, tri, vg))
              {
                *seedcent = pball_center;
                return tri;
              }
            }
          
        }
      }
    }
  }
  return NULL;
}

double get_theta(pcl::PointXYZ cnew, pcl::PointXYZ c, pcl::PointXYZ _m, Edge *e) {
  Eigen::Vector3d c_new(cnew.x, cnew.y, cnew.z);
  Eigen::Vector3d c_old(c.x, c.y, c.z);
  Eigen::Vector3d m(_m.x, _m.y, _m.z);
  Eigen::Vector3d ei_ej(e->v_i->pos.x - e->v_j->pos.x,
                        e->v_i->pos.y - e->v_j->pos.y,
                        e->v_i->pos.z - e->v_i->pos.z);


  Eigen::Vector3d c_new_m = (c_new - m);
  c_new_m.normalize();
  Eigen::Vector3d c_m = (c_old - m);
  c_m.normalize();
  double theta = acos(c_new_m.dot(c_m));
  if ( (c_old - m).cross((c_new - m)).dot(ei_ej) < 0) {
    theta = 2 * M_PI - theta;
  }
  return theta;
}

Vertex *find_candidate(Edge *edge, VoxelGrid *vg, pcl::visualization::PCLVisualizer *viewer, pcl::PointXYZ *cent) {
  Vertex *m = new Vertex();
  m->pos = pcl::PointXYZ(0.5 * (edge->v_i->pos.x + edge->v_j->pos.x),
                         0.5 * (edge->v_i->pos.y + edge->v_j->pos.y),
                         0.5 * (edge->v_i->pos.z + edge->v_j->pos.z)
                         );
  Eigen::Vector3d m_e(m->pos.x, m->pos.y, m->pos.z);
  Eigen::Vector3d c_e(edge->c.x, edge->c.y, edge->c.z);
  std::vector<Vertex*> *neighborhood = vg->findNeighborhood(m);
  std::sort(neighborhood->begin(), neighborhood->end(), [m](const Vertex* lhs, const Vertex* rhs){ return pcl::euclideanDistance(m->pos, lhs->pos) < pcl::euclideanDistance(m->pos, rhs->pos);});
  double theta_min = 2 * M_PI;
  Vertex *candidate = NULL;
  for (Vertex *v_x : *neighborhood) {
    if (edge->v_i == v_x || edge->v_j == v_x || edge->v_o == v_x) {
      continue;
    }
    Triangle *tri = new Triangle();
    tri->points = std::vector<Vertex*>{v_x, edge->v_i, edge->v_j};
    pcl::PointXYZ tri_normal = get_outward_facing_normal(tri);
    double radius = calc_radius (tri);
    double normal_dist = p * p - radius * radius;
    if (normal_dist > 0) {
      pcl::PointXYZ c = calc_circ_center (tri);
      pcl::PointXYZ pball_center(c.x + sqrt(abs(normal_dist)) * tri_normal.x, c.y + sqrt(abs(normal_dist)) * tri_normal.y, c.z + sqrt(abs(normal_dist)) * tri_normal.z);

      double theta = get_theta(pball_center, edge->c, m->pos, edge);
      if (contains_no_other_data_points(pball_center, radius, tri, vg) and theta < theta_min) {
        candidate = v_x;
        theta_min = theta;
        *cent = pball_center;
      }
    } else {
      continue;
    }
  }

  return candidate;
}


/*------------------------ DRAWING RELATED FUNCTIONS ------------------------*/

void Voxel::drawRect(pcl::visualization::PCLVisualizer *viewer)
{
  viewer->addLine(origin, pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "1");
  viewer->addLine(origin, pcl::PointXYZ(origin.x, origin.y + get_delta(), origin.z), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "2");
  viewer->addLine(origin, pcl::PointXYZ(origin.x, origin.y, origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "3");

  viewer->addLine(pcl::PointXYZ(origin.x, origin.y + get_delta(), origin.z), pcl::PointXYZ(origin.x, origin.y + get_delta(), origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "4");
  viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z + get_delta()), pcl::PointXYZ(origin.x, origin.y + get_delta(), origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "5");
  viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z + get_delta()), pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "6");
  viewer->addLine(pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z), pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "7");
  viewer->addLine(pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z+get_delta()), pcl::PointXYZ(origin.x + get_delta(), origin.y + get_delta(), origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "8");
  viewer->addLine(pcl::PointXYZ(origin.x + get_delta(), origin.y+get_delta(), origin.z), pcl::PointXYZ(origin.x + get_delta(), origin.y + get_delta(), origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "9");
  viewer->addLine(pcl::PointXYZ(origin.x, origin.y+get_delta(), origin.z+get_delta()), pcl::PointXYZ(origin.x + get_delta(), origin.y + get_delta(), origin.z + get_delta()), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "10");
  viewer->addLine(pcl::PointXYZ(origin.x, origin.y + get_delta(), origin.z), pcl::PointXYZ(origin.x+get_delta(), origin.y + get_delta(), origin.z), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "11");
  viewer->addLine(pcl::PointXYZ(origin.x + get_delta(), origin.y, origin.z), pcl::PointXYZ(origin.x+get_delta(), origin.y + get_delta(), origin.z), 0.0, 0.0, 1.0, "rect" + std::to_string(origin.x) + std::to_string(origin.y) + std::to_string(origin.z) + "12");
}

void draw_voxel_neighborhood(Vertex *v, VoxelGrid *vg, pcl::visualization::PCLVisualizer *viewer) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr signle (new pcl::PointCloud<pcl::PointXYZRGB>);
  uint8_t r(255), g(15), b(15);
  uint32_t rgb_v = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  pcl::PointXYZRGB point;
  point.x = v->pos.x;
  point.y = v->pos.y;
  point.z = v->pos.z;
  point.rgb = *reinterpret_cast<float*>(&rgb_v);
  signle->push_back(point);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (signle);
  viewer->addPointCloud<pcl::PointXYZRGB>(signle, rgb, "locus");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "locus");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbors (new pcl::PointCloud<pcl::PointXYZRGB>);
  uint8_t r2(15), g2(15), b2(255);
  uint32_t rgb_v2 = (static_cast<uint32_t>(r2) << 16 |
                     static_cast<uint32_t>(g2) << 8 | static_cast<uint32_t>(b2));
  for (Vertex *v : *vg->findNeighborhood(v))
  {
    pcl::PointXYZRGB point;
    point.x = v->pos.x;
    point.y = v->pos.y;
    point.z = v->pos.z;
    point.rgb = *reinterpret_cast<float*>(&rgb_v2);
    neighbors->push_back(point);
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (neighbors);
  viewer->addPointCloud<pcl::PointXYZRGB>(neighbors, rgb2, "locuses");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "locuses");
}

/*------------------------ NORMAL ESTIMATION ------------------------*/

pcl::PointCloud<pcl::Normal>::Ptr find_point_cloud_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, double radius) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (point_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch (radius);
  ne.compute (*cloud_normals);
  return cloud_normals;
}

/*------------------------ p ESTIMATION ------------------------*/

double find_distance_to_nearest(pcl::PointXYZ p, pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree, int K) {
  float min_dist = 0.0;
  K = std::max(K, 16);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  
  if ( kdtree->nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      min_dist += sqrt( pointNKNSquaredDistance[i] ) ;
  }
  return min_dist / (K);
}

/*------------------------ MAIN LOOP ------------------------*/

int main(int argc, char* argv[])
{
  // Read in the point cloud from the source .ply file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader Reader;
  Reader.read(argv[1], *cloud);
  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  // Calculate inward facing normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals = find_point_cloud_normals(cloud, 16);
  //initialize visualization.

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);

  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
  viewer->setBackgroundColor(0.2, 0.2, 0.2);
  /* POINT CLOUD DRAWING */
  if(draw_points){
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  }
  double p_choose = 0.0;
  pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree = new pcl::KdTreeFLANN<pcl::PointXYZ>();
  
  kdtree->setInputCloud (cloud);
  
  int K = 10;
  int i =0;
  double scale = 0.015;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  for (pcl::PointXYZ p : *cloud) {
    /* NORMAL DRAWING */
    p_choose += find_distance_to_nearest(p, kdtree, ceil(cloud->size() * 0.0001 ));
    i++;
  }
  p = (p_choose / cloud->size()) ;
  if (argc == 3) {
    p = std::stod(argv[2]);
  }
  std::cout << "P-VALUE: " << p << "\n";
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
  VoxelGrid *vg = new VoxelGrid();
  max_point_AABB = pcl::PointXYZ(max_point_AABB.x + p * 0.01, max_point_AABB.y + p * 0.01, max_point_AABB.z + p * 0.01);
  min_point_AABB = pcl::PointXYZ(min_point_AABB.x - p * 0.01, min_point_AABB.y - p * 0.01, min_point_AABB.z - p * 0.01);
  boundary_edges = new std::vector<std::vector<Vertex*>*>;
  vg->num_x = ceil((max_point_AABB.x - min_point_AABB.x)/ get_delta());
  vg->num_y = ceil((max_point_AABB.y - min_point_AABB.y)/ get_delta());
  vg->num_z = ceil((max_point_AABB.z - min_point_AABB.z)/ get_delta());
  for (double x = min_point_AABB.x; x < max_point_AABB.x; x += get_delta())
  {
    for (double y = min_point_AABB.y; y < max_point_AABB.y; y += get_delta())
    {

      for (double z = min_point_AABB.z; z < max_point_AABB.z; z += get_delta())
      {
        Voxel *rect = new Voxel();
        rect->origin = pcl::PointXYZ(x, y, z);
        rect->vertices = new std::vector<Vertex*>();
        vg->voxels.push_back(rect);
        /* DRAW VOXELS */
        //rect->drawRect(viewer);
      }
    }
  }
  std::vector<Vertex*> *vertices = new std::vector<Vertex*>();
  int c = 0;
  for (pcl::PointXYZ p : cloud->points) {
    Vertex *v = new Vertex();
    v->used = false;
    v->pos = p;
    pcl::Normal normal = normals->at(c);
    normal.normal_x = -1.0 * normal.normal_x;
    normal.normal_y = -1.0 * normal.normal_y;
    normal.normal_z = -1.0 * normal.normal_z;
    if(draw_normals){
      viewer->addLine(p, pcl::PointXYZ(p.x + scale * normal.normal_x, p.y + scale * normal.normal_y, p.z + scale * normal.normal_z), 0.0, 1.0, 0.0, "norm" + std::to_string(c));
    }
    v->normal = normal;
    vertices->push_back(v);
    v->edges = new std::vector<Vertex*>();
    c++;
  }
  for (Vertex *v : *vertices) {
    vg->findVoxel((v->pos.x - min_point_AABB.x) / get_delta(), (v->pos.y - min_point_AABB.y) / get_delta(), (v->pos.z - min_point_AABB.z) / get_delta())->vertices->push_back(v);
  }
  triangles = new std::vector<Triangle*>();
  if(draw_neighborhood){
  draw_voxel_neighborhood(vertices->at(2000), vg, viewer);
  }
  front = new Front();
  pcl::PointXYZ *cent = new pcl::PointXYZ();
  front->front = new std::vector<Edge*>();
  while (true) {
  //  while (!viewer->wasStopped()) {
  //viewer->spinOnce (0.1);
   Edge *e;
   while((e = get_active_edge()) != NULL) {
   //viewer->spinOnce (0.1);
   Vertex *v = find_candidate(e, vg, viewer, cent);
   if (v != NULL && (!v->used || front->onFront(v))) {
     Triangle *tri = new Triangle();
     tri->points = std::vector<Vertex*>{e->v_i, v, e->v_j};
     tri->points[0]->used = true;
     tri->points[1]->used = true;
     tri->points[2]->used = true;
     triangles->push_back(tri);
     std::cout << "     EXPANDED TRIANGULATION: " << tri->points[0]->to_string() << tri->points[1]->to_string()  << tri->points[2]->to_string() << "\n";
     //tri->drawTriangle(viewer);
     front->join(e, v, cent);
     e->v_j->edges->push_back(e->v_i);
   } else {
     marked_boundary(e);
     continue;
   }

   }
  pcl::PointXYZ *seed_cent = new pcl::PointXYZ();
  if (Triangle *tri = seed_triangle(vertices, vg, viewer, seed_cent))
  {
    tri->points[0]->used = true;
    tri->points[1]->used = true;
    tri->points[2]->used = true;
    triangles->push_back(tri);
    
    std::cout << "ADDED SEED TRIANGLE: " << tri->points[0]->to_string() << tri->points[1]->to_string()  << tri->points[2]->to_string() << "\n";
    //tri->drawTriangle(viewer);
    Edge *e1 = new Edge(tri->points[0], tri->points[1], tri->points[2], *seed_cent);
    front->front->push_back(e1);
    e1->v_i->edges->push_back(e1->v_j);

    Edge *e2 = new Edge(tri->points[1], tri->points[2], tri->points[0], *seed_cent);
    front->front->push_back(e2);
    e2->v_i->edges->push_back(e2->v_j);

    Edge *e3 = new Edge(tri->points[2], tri->points[0], tri->points[1], *seed_cent);
    front->front->push_back(e3);
    e3->v_i->edges->push_back(e3->v_j);
  } else {
      break;
  }
  }
  std::cout << "MESH COMPLETED WITH " << triangles->size() << " TRIANGLES\n";
  for (Triangle *t : *triangles) {
    t->drawTriangle(viewer);
  }
  while (!viewer->wasStopped ()) {
    
    viewer->spinOnce (1);
  }

  return 0;
}
