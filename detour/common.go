package detour

import (
	"github.com/johanhenriksson/goworld/math"
	"github.com/johanhenriksson/goworld/math/vec3"
)

// Computational geometry helper functions.

// TriArea2D derives the signed xz-plane area of the triangle ABC, or the
// relationship of line AB to point C.
//
//	a   Vertex A. [(x, y, z)]
//	b   Vertex B. [(x, y, z)]
//	c   Vertex C. [(x, y, z)]
//
// return The signed xz-plane area of the triangle.
func TriArea2D(a, b, c vec3.T) float32 {
	abx := b.X - a.X
	abz := b.Z - a.Z
	acx := c.X - a.X
	acz := c.Z - a.Z
	return acx*abz - abx*acz
}

// IntersectSegSeg2D returns whether two segments intersect, and their
// intersection point.
func IntersectSegSeg2D(ap, aq, bp, bq vec3.T) (hit bool, s, t float32) {
	u := aq.Sub(ap)
	v := bq.Sub(bp)
	w := ap.Sub(bp)

	d := perp2D(u, v)
	if math.Abs(d) < 1e-6 {
		return false, s, t
	}
	return true, perp2D(v, w) / d, perp2D(u, w) / d
}

// OverlapQuantBounds determines if two axis-aligned bounding boxes overlap.
//
//	amin   Minimum bounds of box A. [(x, y, z)]
//	amax   Maximum bounds of box A. [(x, y, z)]
//	bmin   Minimum bounds of box B. [(x, y, z)]
//	bmax   Maximum bounds of box B. [(x, y, z)]
//	return True if the two AABB's overlap.
//
// see overlapBounds
func OverlapQuantBounds(amin, amax, bmin, bmax []uint16) bool {
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		return false
	}

	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		return false
	}

	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		return false
	}
	return true
}

// OverlapBounds determines if two axis-aligned bounding boxes overlap.
//
//	Arguments:
//	 amin     Minimum bounds of box A. [(x, y, z)]
//	 amax     Maximum bounds of box A. [(x, y, z)]
//	 bmin     Minimum bounds of box B. [(x, y, z)]
//	 bmax     Maximum bounds of box B. [(x, y, z)]
//
// Return True if the two AABB's overlap.
// see overlapQuantBounds
func OverlapBounds(amin, amax, bmin, bmax vec3.T) bool {
	if amin.X > bmax.X || amax.X < bmin.X {
		return false
	}
	if amin.Y > bmax.Y || amax.Y < bmin.Y {
		return false
	}
	if amin.Z > bmax.Z || amax.Z < bmin.Z {
		return false
	}
	return true
}

func distancePtPolyEdgesSqr(pt vec3.T, verts []vec3.T, ed, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := 0, (len(verts) - 1); i < len(verts); i++ {
		vi := verts[i]
		vj := verts[j]
		if ((vi.Z > pt.Z) != (vj.Z > pt.Z)) &&
			(pt.X < (vj.X-vi.X)*(pt.Z-vi.Z)/(vj.Z-vi.Z)+vi.X) {
			c = !c
		}
		ed[j] = distancePtSegSqr2D(pt, vj, vi, &et[j])
		j = i
	}
	return c
}

func perp2D(v, u vec3.T) float32 {
	return v.Z*u.X - v.X*u.Z
}

func IntersectSegmentPoly2D(p0, p1 vec3.T, verts []vec3.T) (tmin, tmax float32, segMin, segMax int, res bool) {
	const eps float32 = 0.00000001

	tmin = 0
	tmax = 1
	segMin = -1
	segMax = -1

	dir := p1.Sub(p0)
	j := len(verts) - 1
	for i := 0; i < len(verts); i++ {
		edge := verts[i].Sub(verts[j])
		diff := p0.Sub(verts[j])
		n := perp2D(edge, diff)
		d := perp2D(dir, edge)
		if math.Abs(d) < eps {
			// S is nearly parallel to this edge
			if n < 0 {
				return
			}
			j = i
			continue
		}
		t := n / d
		if d < 0 {
			// segment S is entering across this edge
			if t > tmin {
				tmin = t
				segMin = j
				// S enters after leaving polygon
				if tmin > tmax {
					return
				}
			}
		} else {
			// segment S is leaving across this edge
			if t < tmax {
				tmax = t
				segMax = j
				// S leaves before entering polygon
				if tmax < tmin {
					return
				}
			}
		}
		j = i
	}

	res = true
	return
}

func distancePtSegSqr2D(pt, p, q vec3.T, t *float32) float32 {
	pqx := q.X - p.X
	pqz := q.Z - p.Z
	dx := pt.X - p.X
	dz := pt.Z - p.Z
	d := pqx*pqx + pqz*pqz
	*t = pqx*dx + pqz*dz
	if d > 0 {
		*t /= d
	}
	if *t < float32(0) {
		*t = 0
	} else if *t > 1 {
		*t = 1
	}
	dx = p.X + *t*pqx - pt.X
	dz = p.Z + *t*pqz - pt.Z
	return dx*dx + dz*dz
}

func dot2D(v, u vec3.T) float32 {
	return v.X*u.X + v.Z*u.Z
}

func closestHeightPointTriangle(p, a, b, c vec3.T) (h float32, ok bool) {
	v0 := c.Sub(a)
	v1 := b.Sub(a)
	v2 := p.Sub(a)

	dot00 := dot2D(v0, v0)
	dot01 := dot2D(v0, v1)
	dot02 := dot2D(v0, v2)
	dot11 := dot2D(v1, v1)
	dot12 := dot2D(v1, v2)

	// Compute barycentric coordinates
	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	EPS := float32(1e-4)

	// If point lies inside the triangle, return interpolated ycoord.
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		h = a.Y + v0.Y*u + v1.Y*v
		return h, true
	}

	return 0, false
}

func oppositeTile(side int32) int32 {
	return (side + 4) & 0x7
}
