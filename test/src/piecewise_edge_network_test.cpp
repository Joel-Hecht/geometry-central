#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/surface/meshio.h>
#include <geometrycentral/utilities/vector3.h>
#include "geometrycentral/surface/flip_geodesics.h"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include "load_test_meshes.h"

#include "gtest/gtest.h"

#include <string>
#include <iostream>
#include <memory>

using namespace geometrycentral;
using namespace geometrycentral::surface;
using std::cout;
using std::endl;

class pieceWiseEdgeNetworkTest : public MeshAssetSuite {};

TEST_F(pieceWiseEdgeNetworkTest, deletesSingleDetour) {
	for(auto& asset : {getAsset("dijkstra.obj", true)}) {
		Eigen::MatrixXd out;
		ManifoldSurfaceMesh& mesh = *asset.manifoldMesh; 
		std::unique_ptr<geometrycentral::surface::FlipEdgeNetwork> network;
    VertexPositionGeometry& origGeometry = *asset.geometry;
		std::vector<Vertex> path;
		std::vector<int> points = {0, 2, 3};

		for(size_t i = 0; i < points.size(); i++) {
			path.push_back(mesh.vertex(points[i]));
		}

		network = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(mesh, origGeometry, path, false);
		for(auto &eptr : network->paths){
			auto x = eptr->getHalfedgeList();
			for(auto he : x) {
				std::cout << he.vertex() << std::endl;
			}
			EXPECT_EQ(x.size(), 2);
		}

	}
			
}

TEST_F(pieceWiseEdgeNetworkTest, deleteMultipleAtStart) {
	for(auto& asset : {getAsset("dijkstra.obj", true)}) {
		Eigen::MatrixXd out;
		ManifoldSurfaceMesh& mesh = *asset.manifoldMesh; 
		std::unique_ptr<geometrycentral::surface::FlipEdgeNetwork> network;
    VertexPositionGeometry& origGeometry = *asset.geometry;
		std::vector<Vertex> path;
		std::vector<int> points = {1, 5, 3};

		for(size_t i = 0; i < points.size(); i++) {
			path.push_back(mesh.vertex(points[i]));
		}

		network = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(mesh, origGeometry, path, false);
		for(auto &eptr : network->paths){
			EXPECT_EQ(eptr->getHalfedgeList().size(), 1);
		}

	}
			
}

TEST_F(pieceWiseEdgeNetworkTest, deleteMultipleAtEnd) {
	for(auto& asset : {getAsset("dijkstra.obj", true)}) {
		Eigen::MatrixXd out;
		ManifoldSurfaceMesh& mesh = *asset.manifoldMesh; 
		std::unique_ptr<geometrycentral::surface::FlipEdgeNetwork> network;
    VertexPositionGeometry& origGeometry = *asset.geometry;
		std::vector<Vertex> path;
		std::vector<int> points = {0, 2, 5, 1};

		for(size_t i = 0; i < points.size(); i++) {
			path.push_back(mesh.vertex(points[i]));
		}

		network = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(mesh, origGeometry, path, false);
		for(auto &eptr : network->paths){
			EXPECT_EQ(eptr->getHalfedgeList().size(), 1);
		}
	}
			
}
