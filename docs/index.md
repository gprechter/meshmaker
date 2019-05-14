#  Final Project - Mesh Maker

## Abstract

In our project, Mesh Maker, we implemented the Ball-Pivoting Algorithm (BPA) to create a triangle-mesh from a 3D object represented by a point-cloud. In our implementation we referenced multiple research papers on the algorithm such as [The Ball-Pivoting Algorithm for Surface Reconstruction](https://lidarwidgets.com/samples/bpa_tvcg.pdf) and [An Analysis and Implementation of a Parallel Ball Pivoting Algorithm](https://pdfs.semanticscholar.org/8ec0/d70299f83ccb98ad593a1b581deb018cbfe2.pdf). Point-clouds are a common source of output data from 3D scanners, yet lack the surface information and topographical relationships between points found in meshes. Creating an accurate mesh reconstruciton of a point-cloud for rendering and digital modeling is the drive behind implementing a program to create a triangle-mesh from a point-cloud.

Th Ball-Pivoting Algorithm treats the point cloud $PC$ as a sampling of some manifold surface $MS$. A ball (known as a $p$-ball for having a radius of $p$) is created such that it would not fall through the surface of the object represented by $PC$. This ball is placed to rest on the mesh on a trio of points, and essentially 'rolls' or 'pivots' across the edges of the reconstructed portion of the mesh to add new triangles as it discovers new points. In the end, when no new triangles can be added by the algorithm, a Triangle mesh is output. Since BPA is quite a visual algorithm, one of the key concerns for our implementation was allowing for the algorithm to be visualized, so that we could watch the ball 'roll' over the surface. We utilized [PCL](http://pointclouds.org), an open source library for reading point clouds and visualizing them.

## Implementation





















<div align="middle">
        <img src="images/p1/f1.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a head on view</figcaption>
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p1/f2.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a different view</figcaption>
      </td>
        <td>
        <img src="images/p1/f3.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a third view</figcaption>
      </td>
    </tr>
</table>
</div>