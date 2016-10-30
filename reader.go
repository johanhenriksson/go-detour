package detour

import (
	"encoding/binary"
	"fmt"
	"io"
)

type Reader struct{}

//func (r Reader) Read(p []byte) (n int, err error)

// Decode reads a PNG image from r and returns it as an image.Image.
// The type of Image returned depends on the PNG contents.
func Decode(r io.Reader) (*DtNavMesh, error) {
	// Read header.
	var (
		hdr NavMeshSetHeader
		err error
	)

	err = binary.Read(r, binary.LittleEndian, &hdr)
	if err != nil {
		return nil, err
	}
	fmt.Println(hdr)

	if hdr.Magic != NAVMESHSET_MAGIC {
		return nil, fmt.Errorf("wrong magic number: %x", hdr.Magic)
	}

	if hdr.Version != NAVMESHSET_VERSION {
		return nil, fmt.Errorf("wrong version: %d", hdr.Version)
	}

	var mesh DtNavMesh
	return &mesh, nil
}
