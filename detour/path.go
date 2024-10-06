package detour

import "github.com/johanhenriksson/goworld/math/vec3"

type Path struct {
	Point vec3.T
	Flags uint8
	Poly  PolyRef
}
