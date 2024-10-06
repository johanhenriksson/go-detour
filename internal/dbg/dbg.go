package main

import (
	"fmt"
	"log"
	"os"

	assert "github.com/arl/assertgo"
	"github.com/arl/go-detour/detour"
	"github.com/johanhenriksson/goworld/math/vec3"
)

func check(err error) {
	if err != nil {
		log.Fatalln(err)
		os.Exit(1)
	}
}

func main() {
	var (
		f    *os.File
		err  error
		mesh *detour.NavMesh
	)

	f, err = os.Open("testdata/navmesh.bin")
	check(err)
	defer f.Close()

	mesh, err = detour.Decode(f)
	check(err)
	if mesh == nil {
		fmt.Println("error loading mesh")
		return
	}
	fmt.Println("mesh loaded successfully")
	fmt.Printf("mesh params: %#v\n", mesh.Params)
	fmt.Println("Navigation Query")

	org := vec3.New(3, 0, 1)
	dst := vec3.New(50, 0, 30)

	path, err := findPath(mesh, org, dst)
	if err != nil {
		log.Fatalln("findPath failed", err)
	}
	log.Println("findPath success, path:", path)
}

func findPath(mesh *detour.NavMesh, org, dst vec3.T) ([]detour.PolyRef, error) {
	query, err := detour.NewNavMeshQuery(mesh, 1000)
	if err != nil {
		return nil, fmt.Errorf("query creation failed with error %s\n", err)
	}
	// define the extents vector for the nearest polygon query
	extents := vec3.New(0, 2, 0)

	// create a default query filter
	filter := detour.NewStandardQueryFilter()

	// get org polygon reference
	orgRef, nearestPt, err := query.FindNearestPoly(org, extents, filter)
	if err != nil {
		return nil, fmt.Errorf("FindNearestPoly failed with %v\n", err)
	} else if orgRef == 0 {
		return nil, fmt.Errorf("org doesn't intersect any polygons")
	}
	assert.True(mesh.IsValidPolyRef(orgRef), "%d is not a valid poly ref")
	org = nearestPt
	log.Println("org is now", org)

	// get dst polygon reference
	dstRef, nearestPt, err := query.FindNearestPoly(dst, extents, filter)
	if err != nil {
		return nil, fmt.Errorf("FindNearestPoly failed with %v\n", err)
	} else if dstRef == 0 {
		return nil, fmt.Errorf("dst doesn't intersect any polygons")
	}
	assert.True(mesh.IsValidPolyRef(orgRef), "%d is not a valid poly ref")
	dst = nearestPt
	log.Println("dst is now", dst)

	// FindPath
	path := make([]detour.PolyRef, 100)
	path, err = query.FindPath(orgRef, dstRef, org, dst, filter, path)
	if err != nil {
		return path, fmt.Errorf("query.FindPath failed with %v\n", err)
	}
	return path, nil

	//fmt.Println("FindPath", "org:", org, "dst:", dst, "orgRef:", orgRef, "dstRef:", dstRef)
	//fmt.Println("FindPath set pathCount to", pathCount)
	//fmt.Println("path", path)
	//fmt.Println("actual path returned", path[:pathCount])

	//// If the end polygon cannot be reached through the navigation graph,
	//// the last polygon in the path will be the nearest the end polygon.
	//// check for that
	//if path[len(path)-1] == dstRef {
	//fmt.Println("no path found, as last poly in path in dstPoly")
	//} else {
	//fmt.Println("path found")
	//for _, polyRef := range path[:pathCount] {
	//fmt.Println("-poly ref", polyRef)
	//mesh.TileAndPolyByRefUnsafe(polyRef, &ptile, &ppoly)
	//polyIdx := mesh.DecodePolyIdPoly(polyRef)
	//poly := ptile.Polys[polyIdx]

	//centroid := make([]float32, 3)
	//detour.CalcPolyCenter(centroid, poly.Verts[:], int32(poly.VertCount), ptile.Verts)
	//fmt.Println("poly center: ", centroid)

	////for _, v := range poly.Verts[0:poly.VertCount] {
	////fmt.Println("poly vertex", ptile.Verts[3*v:3*v+3])
	////}
	//}
	//}
	return path, nil
}
