package detour

import (
	"fmt"
	"log"
	"reflect"
	"testing"

	"github.com/johanhenriksson/goworld/math/vec3"
)

func TestOffMeshConnections(t *testing.T) {
	t.Skip("Work in progress, disabled for now")
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst           vec3.T
		incFlags, excFlags uint16 // query filter include/exclude flags
		wantPath           []PolyRef
		wantStraightPath   []vec3.T
	}{
		{
			// latest result:
			/*
			   ps  -19.460140 4.234787 -4.727699  -1.402759 -0.000092 -2.314920  0xffef 0x0
			           findPath polys[0] ref:0x600029
			           findPath polys[1] ref:0x60003d
			           findPath polys[2] ref:0x600034
			           ps[0] pt{-19.460140,4.234787,-4.727699} ref:0x600029 flags0x1
			           ps[1] pt{-17.767578,2.514686,-0.300116} ref:0x60003d flags0x4
			           ps[2] pt{-6.194458,0.197294,1.019781} ref:0x600034 flags0x0
			           ps[3] pt{-1.402759,-0.000092,-2.314920} ref:0x0 flags0x2
			*/

			// oldest result:
			//offmeshconnections (omc) are currently only allowed inside a same tile or its direct neighbours :-(

			// with the new and already committed navmesh 'offmeshcons.bin', we
			// got this result on RecastDemo gui:
			//ps  -17.323172 3.074387 -3.366402  -4.377790 -0.000053 -1.633406  0xffef 0x0
			//p[0] pt{-17.323172,3.074387,-3.366402} flags0x1
			//p[1] pt{-17.767578,2.514686,-0.300116} flags0x4
			//p[2] pt{-6.194458,0.197294,1.019781} flags0x0
			//p[3] pt{-4.377790,-0.000053,-1.633406} flags0x2

			//d3.Vec3{13.512327, 9.998181, -39.930054},
			//d3.Vec3{20.308548, 11.554298, -57.635326},
			//d3.Vec3{-17.323172, 3.074387, -3.366402},
			//d3.Vec3{-4.377790, -0.000053, -1.633406},
			vec3.New(-19.460140, 4.234787, -4.727699),
			vec3.New(-1.402759, -0.000092, -2.314920),

			0xffef, // all poly but the disabled ones
			0x0,
			//[]PolyRef{0x600029, 0x600023, 0x600025, 0x60001d, 0x60001c, 0x60001e, 0x600020, 0x60001f, 0x600016, 0x600012, 0x60000b, 0x600018, 0x600015, 0x600006, 0x600005, 0x600007, 0x600008, 0x600034},
			[]PolyRef{0x600029, 0x60003d, 0x600034},
			[]vec3.T{
				vec3.New(-19.460140, 4.234787, -4.727699),
				vec3.New(-17.767578, 2.514686, -0.300116),
				vec3.New(-6.194458, 0.197294, 1.019781),
				vec3.New(-1.402759, -0.000092, -2.314920),
			},

			/*         []d3.Vec3{*/
			//d3.NewVec3XYZ(5, 0, 10),
			//d3.NewVec3XYZ(3.900252, 0.189468, 11.998747),
			//d3.NewVec3XYZ(14.700253, 0.189468, 19.198748),
			//d3.NewVec3XYZ(15.900252, 0.189468, 19.198748),
			//d3.NewVec3XYZ(24.3, 0.189468, 28.798748),
			//d3.NewVec3XYZ(31.8, 0.189468, 32.098747),
			//d3.NewVec3XYZ(39.9, 0.189468, 32.098747),
			//d3.NewVec3XYZ(50, 0, 30),
			/*},*/
		},
	}

	mesh, err = loadTestNavMesh("offmeshcons.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			query          *NavMeshQuery        // the query instance
			filter         *StandardQueryFilter // the query filter
			orgRef, dstRef PolyRef              // find poly query results
			org, dst       vec3.T               // find poly query results
			st             Status               // status flags
			path           []PolyRef            // returned path
		)

		st, query = NewNavMeshQuery(mesh, 1000)
		if StatusFailed(st) {
			t.Error("query creation failed:", st)
		}
		// define the extents vector for the nearest polygon query
		extents := vec3.New(2, 2, 2)

		// create a default query filter
		filter = NewStandardQueryFilter()
		filter.SetIncludeFlags(tt.incFlags)
		filter.SetExcludeFlags(tt.excFlags)

		// These are just sample areas to use consistent values across the samples.
		// The use should specify these base on his needs.
		const (
			samplePolyAreaGround int32 = iota
			samplePolyAreaWater
			samplePolyAreaRoad
			samplePolyAreaDoor
			samplePolyAreaGrass
			samplePolyAreaJump
		)

		const (
			samplePolyFlagsWalk     int32 = 0x01   // Ability to walk (ground, grass, road)
			samplePolyFlagsSwim     int32 = 0x02   // Ability to swim (water).
			samplePolyFlagsDoor     int32 = 0x04   // Ability to move through doors.
			samplePolyFlagsJump     int32 = 0x08   // Ability to jump.
			samplePolyFlagsDisabled int32 = 0x10   // Disabled polygon
			samplePolyFlagsAll      int32 = 0xffff // All abilities.
		)

		// Change costs.
		filter.SetAreaCost(samplePolyAreaGround, 1.0)
		filter.SetAreaCost(samplePolyAreaWater, 10.0)
		filter.SetAreaCost(samplePolyAreaRoad, 1.0)
		filter.SetAreaCost(samplePolyAreaDoor, 1.0)
		filter.SetAreaCost(samplePolyAreaGrass, 2.0)
		filter.SetAreaCost(samplePolyAreaJump, 1.5)

		// get org polygon reference
		st, orgRef, org = query.FindNearestPoly(tt.org, extents, filter)
		if StatusFailed(st) {
			t.Fatal("couldn't find nearest poly of", tt.org, ":", st)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatal("orgRef", orgRef, "is not a valid poly ref")
		}

		t.Logf("org poly reference:0x%x\n", orgRef)

		// get dst polygon reference
		st, dstRef, dst = query.FindNearestPoly(tt.dst, extents, filter)
		if StatusFailed(st) {
			t.Fatal("couldn't find nearest poly of", tt.dst, ":", st)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatal("dstRef", dstRef, "is not a valid poly ref")
		}

		t.Logf("dst poly reference:0x%x\n", dstRef)

		// FindPath
		var (
			pathCount int
		)
		path = make([]PolyRef, 100)
		pathCount, st = query.FindPath(orgRef, dstRef, org, dst, filter, path)
		if StatusFailed(st) {
			t.Fatal("query.FindPath failed:", st)
		}

		if !reflect.DeepEqual(tt.wantPath, path[:pathCount]) {
			t.Fatalf("found path is not correct, want %#v, got %#v", tt.wantPath, path[:pathCount])
		}

		// FindStraightPath
		var (
			straightPath      []vec3.T
			straightPathFlags []uint8
			straightPathRefs  []PolyRef
			straightPathCount int
			maxStraightPath   int32
		)
		// slices that receive the straight path
		maxStraightPath = 100
		straightPath = make([]vec3.T, maxStraightPath)
		straightPathFlags = make([]uint8, maxStraightPath)
		straightPathRefs = make([]PolyRef, maxStraightPath)

		straightPathCount, st = query.FindStraightPath(tt.org, tt.dst, path[:pathCount], straightPath, straightPathFlags, straightPathRefs, 0)
		if StatusFailed(st) {
			t.Fatal("query.FindStraightPath failed:", st)
		}

		if (straightPathFlags[0] & StraightPathStart) == 0 {
			t.Fatal("straightPath start is not flagged StraightPathStart")
		}

		fmt.Println("StraightPathCount == ", straightPathCount)
		if (straightPathFlags[straightPathCount-1] & StraightPathEnd) == 0 {
			t.Fatal("straightPath end is not flagged StraightPathEnd")
		}

		if int(straightPathCount) != len(tt.wantStraightPath) {
			t.Fatalf("found path and wanted path do not have the same length (%d != %d)", straightPathCount, len(tt.wantStraightPath))
		}
		for i := 0; i < straightPathCount; i++ {
			log.Printf("straightPath[%d].Flags = 0x%x\n", i, straightPathFlags[i])
		}
		for i := 0; i < pathCount; i++ {
			if !straightPath[i].ApproxEqual(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}
	}
}
