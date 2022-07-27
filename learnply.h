/*

Data structures for learnply

Eugene Zhang 2005

*/

#ifndef __LEARNPLY_H__
#define __LEARNPLY_H__

#include "rtweekend.h"
#include <vector>
#include "ply.h"

/* forward declarations */
class Triangle;
class Vertex {
public:
  int index;
  vec3 pos;
  vec3 normal;

  std::vector<Triangle*> tris;

  void *other_props;

public:
	Vertex(vec3& p) : index(-1), pos(p), normal(vec3(0.0, 0.0, 1.0)) {  }
	Vertex(double xx, double yy, double zz): index(-1), pos(vec3(xx, yy, zz)), normal(vec3(0.0, 0.0, 1.0)) {  }

	int ntris() { return (int)tris.size(); }
};

class Edge 
{
public:
  int index;

  Vertex *verts[2];

  std::vector<Triangle*> tris;

  double length;

public:
	Edge() : index(-1), length(0.0), verts{NULL, NULL} { }
	int ntris() { return (int)tris.size(); }
};

class Triangle {
public:
  int index;

  vec3 normal;
  double  area;

  int nverts;
  Vertex *verts[3];
  Edge   *edges[3];

  void *other_props; 

public:
	Triangle() : index(-1), nverts(3), normal(vec3(0.0, 0.0, 1.0)), area(0.0), 
		verts{NULL, NULL, NULL}, edges{ NULL, NULL, NULL }
	{  }
};

class Polyhedron 
{
public:
	PlyFile* in_ply;
	std::vector<Triangle*> tlist;/* list of triangles */
	std::vector<Vertex*>   vlist;/* list of vertices */
	std::vector<Edge*>     elist;/* list of edges */

	unsigned char orientation;  // 0=ccw, 1=cw
	vec3 center;
	double radius;
	double area;

	int seed;//selection

public:

	Polyhedron();
	Polyhedron(std::vector<Vertex*>& verts, std::vector<Triangle*>& tris, bool re_index = true);
	Polyhedron(FILE*);
	//
	int ntris() { return (int)tlist.size(); }
	int nverts() { return (int)vlist.size(); }
	int nedges() { return (int)elist.size(); }
	// initialization and finalization
	void initialize();
	void finalize();
private:
	  PlyOtherProp *vert_other, *face_other;
	  void write_file(FILE*);

	  void calc_vert_normals();

	  void create_edge(Vertex *, Vertex *);
	  void create_edges();

	  int face_to_vertex_ref(Triangle *, Vertex *);
	  void order_vertex_to_tri_ptrs(Vertex *);
	  void vertex_to_tri_ptrs();

	  Triangle *find_common_edge(Triangle *, Vertex *, Vertex *);
	  Triangle *other_triangle(Edge *, Triangle *);

	  void calc_bounding_sphere();
	  void calc_face_normals_and_area();
	  void calc_edge_length();

	  void create_pointers();
};

#endif /* __LEARNPLY_H__ */

