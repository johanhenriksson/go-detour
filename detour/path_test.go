package detour

import (
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"testing"

	"github.com/johanhenriksson/goworld/math/vec3"
)

func checkt(t *testing.T, err error) {
	if err != nil {
		t.Fatalf("fail with error: %v", err)
	}
}

func loadTestNavMesh(fname string) (*NavMesh, error) {
	f, err := os.Open(filepath.Join("..", "testdata", fname))
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Decode(f)
}

func TestFindPathFindStraightPath(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		org, dst         vec3.T
		wantPath         []PolyRef
		wantStraightPath []vec3.T
	}{
		{
			vec3.New(37.298489, -1.776901, 11.652311),
			vec3.New(42.457218, 7.797607, 17.778244),
			[]PolyRef{
				0x18c,
				0x18a,
				0x156,
				0x157,
				0x159,
				0x158,
				0x160,
				0x15f,
				0x174,
				0x175,
				0x176,
				0x19d,
				0x19f},
			[]vec3.T{
				vec3.New(37.298489, -1.776901, 11.652311),
				vec3.New(35.310688, -0.469517, 5.899849),
				vec3.New(34.410686, -0.669517, -1.600151),
				vec3.New(35.610683, -0.069517, -1.900150),
				vec3.New(36.510685, 0.730483, -0.400150),
				vec3.New(41.010685, 7.930483, 15.199852),
				vec3.New(42.457218, 7.797607, 17.778244),
			},
		},
	}

	mesh, err = loadTestNavMesh("mesh1.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			path []PolyRef // returned path
		)

		query, err := NewNavMeshQuery(mesh, 1000)
		if errors.Is(err, ErrFailure) {
			t.Fatalf("query creation failed with status 0x%x\n", err)
		}
		// define the extents vector for the nearest polygon query
		extents := vec3.New(2, 4, 2)

		// create a default query filter
		filter := NewStandardQueryFilter()

		// get org polygon reference
		orgRef, org, err := query.FindNearestPoly(tt.org, extents, filter)
		if errors.Is(err, ErrFailure) {
			t.Fatalf("couldn't find nearest poly of %v, error: %s\n", tt.org, err)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Fatalf("orgRef %d is not a valid poly ref", orgRef)
		}
		if orgRef != tt.wantPath[0] {
			t.Fatalf("orgRef %d is not the expected poly ref %d", orgRef, tt.wantPath[0])
		}

		// get dst polygon reference
		dstRef, dst, err := query.FindNearestPoly(tt.dst, extents, filter)
		if errors.Is(err, ErrFailure) {
			t.Fatalf("couldn't find nearest poly of %v, error: %s\n", tt.org, err)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Fatalf("dstRef %d is not a valid poly ref", dstRef)
		}
		if dstRef != tt.wantPath[len(tt.wantPath)-1] {
			t.Fatalf("dstRef %d is not the expected poly ref %d", dstRef, tt.wantPath[len(tt.wantPath)-1])
		}

		// FindPath
		path = make([]PolyRef, 100)
		path, err = query.FindPath(orgRef, dstRef, org, dst, filter, path)
		if errors.Is(err, ErrFailure) {
			t.Fatalf("query.FindPath failed with 0x%x\n", err)
		}

		if !reflect.DeepEqual(tt.wantPath, path) {
			t.Fatalf("found path is not correct, want %#v, got %#v", tt.wantPath, path)
		}

		// FindStraightPath
		var (
			straightPathCount int
			maxStraightPath   int32
		)
		// slices that receive the straight path
		maxStraightPath = 100
		straightPath := make([]Path, maxStraightPath)

		straightPathCount, err = query.FindStraightPath(tt.org, tt.dst, path, straightPath, 0)
		if errors.Is(err, ErrFailure) {
			t.Errorf("query.FindStraightPath failed with %s\n", err)
		}

		if (straightPath[0].Flags & StraightPathStart) == 0 {
			t.Errorf("straightPath start is not flagged StraightPathStart")
		}

		if (straightPath[straightPathCount-1].Flags & StraightPathEnd) == 0 {
			t.Errorf("straightPath end is not flagged StraightPathEnd")
		}

		for i := 0; i < straightPathCount; i++ {
			if !straightPath[i].Point.ApproxEqual(tt.wantStraightPath[i]) {
				t.Errorf("straightPath[%d] = %v, want %v", i, straightPath[i], tt.wantStraightPath[i])
			}
		}
	}
}

func TestFindPathSpecialCases(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		msg           string // test description
		org, dst      vec3.T // path origin and destination points
		wantErr       error  // expected status
		wantPathCount int    // expected path count
	}{
		{"org == dst", vec3.New(5, 0, 10), vec3.New(5, 0, 10), nil, 1},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		query, err := NewNavMeshQuery(mesh, 1000)
		if err != nil {
			t.Errorf("query creation failed with error %s\n", err)
		}
		// define the extents vector for the nearest polygon query
		extents := vec3.New(0, 2, 0)

		// create a default query filter
		filter := NewStandardQueryFilter()

		// get org polygon reference
		orgRef, org, err := query.FindNearestPoly(tt.org, extents, filter)
		if err != nil {
			t.Errorf("couldn't find nearest poly of %v, error: %s\n", tt.org, err)
		}
		if !mesh.IsValidPolyRef(orgRef) {
			t.Errorf("invalid ref (0x%x) for nearest poly of %v, status: %s", orgRef, tt.org, err)
		}

		// get dst polygon reference
		dstRef, dst, err := query.FindNearestPoly(tt.dst, extents, filter)
		if err != nil {
			t.Errorf("couldn't find nearest poly of %v, status: %s\n", tt.org, err)
		}
		if !mesh.IsValidPolyRef(dstRef) {
			t.Errorf("dstRef %d is not a valid poly ref", dstRef)
		}

		// FindPath
		path := make([]PolyRef, 100)
		path, err = query.FindPath(orgRef, dstRef, org, dst, filter, path)

		if err != tt.wantErr {
			t.Errorf("%s, got error %s, want %s", tt.msg, err, tt.wantErr)
		}
		if len(path) != tt.wantPathCount {
			t.Errorf("%s, got pathCount %d, want %d", tt.msg, len(path), tt.wantPathCount)
		}
	}
}
