package detour

import (
	"testing"

	"github.com/johanhenriksson/goworld/math/vec3"
)

func TestCalcPolyCenter(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	polyTests := []struct {
		ref  PolyRef
		want vec3.T
	}{
		{0x440000, vec3.New(3.6002522, 0.189468, 10.873747)},
		{0x460007, vec3.New(11.460253, 0.189468, 14.758746)},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range polyTests {
		tile, poly, status := mesh.TileAndPolyByRef(tt.ref)
		if !StatusSucceed(status) {
			t.Errorf("couldn't retrieve tile and poly for ref 0x%x", tt.ref)
		}
		got := CalcPolyCenter(poly.Verts[:], int32(poly.VertCount), tile.Verts)
		if !got.ApproxEqual(tt.want) {
			t.Errorf("want centroid of poly 0x%x = %v, got %v", tt.ref, tt.want, got)
		}
	}
}

func TestFindNearestPolySpecialCases(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		msg     string  // test description
		pt      vec3.T  // point
		ext     vec3.T  // search extents
		wantSt  Status  // expected status
		wantRef PolyRef // expected ref (if query succeeded)
	}{
		{
			"search box does not intersect any poly",
			vec3.New(-5, 0, 10), vec3.New(1, 1, 1), Success, 0,
		},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for _, tt := range pathTests {
		var (
			q   *NavMeshQuery
			st  Status
			ref PolyRef
			f   QueryFilter
		)

		st, q = NewNavMeshQuery(mesh, 100)
		f = NewStandardQueryFilter()

		st, ref, _ = q.FindNearestPoly(tt.pt, tt.ext, f)
		if st != tt.wantSt {
			t.Errorf("%s, want status 0x%x, got 0x%x", tt.msg, tt.wantSt, st)
		}

		if StatusSucceed(st) && ref != tt.wantRef {
			t.Errorf("%s, want ref 0x%x, got 0x%x", tt.msg, tt.wantRef, ref)
		}
	}
}
