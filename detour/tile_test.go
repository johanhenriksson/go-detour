package detour

import (
	"testing"

	"github.com/johanhenriksson/goworld/math/vec3"
)

func TestFindNearestPolyInTile(t *testing.T) {
	var (
		mesh *NavMesh
		err  error
	)

	pathTests := []struct {
		pt   vec3.T  // point
		ext  vec3.T  // search extents
		want PolyRef // wanted poly ref
	}{
		{
			vec3.New(5, 0, 10),
			vec3.New(0, 1, 0),
			0x440000,
		},
		{
			vec3.New(50, 0, 30),
			vec3.New(1, 0, 1),
			0x620000,
		},
	}

	mesh, err = loadTestNavMesh("mesh2.bin")
	checkt(t, err)

	for i, tt := range pathTests {
		t.Logf("test %d: pt:%v ext:%v", i, tt.pt, tt.ext)

		// calc tile location
		tx, ty := mesh.CalcTileLoc(tt.pt)
		tile := mesh.TileAt(tx, ty, 0)
		if tile == nil {
			t.Errorf("couldn't retrieve tile at point %v", tt.pt)
			continue
		}

		got, _ := mesh.FindNearestPolyInTile(tile, tt.pt, tt.ext)
		if got != tt.want {
			t.Errorf("got polyref 0x%x, want 0x%x", got, tt.want)
			continue
		}
	}
}
