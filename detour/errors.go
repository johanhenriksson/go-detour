package detour

import (
	"errors"
	"fmt"
)

var ErrFailure = errors.New("operation failed")
var ErrWrongMagic = fmt.Errorf("%w: input data is not recognized", ErrFailure)
var ErrWrongVersion = fmt.Errorf("%w: input data is in wrong version", ErrFailure)
var ErrOutOfMemory = fmt.Errorf("%w: operation ran out of memory", ErrFailure)
var ErrInvalidParam = fmt.Errorf("%w: an input parameter was invalid", ErrFailure)

var ErrBufferTooSmall = errors.New("result buffer for the query was too small to store all results")
var ErrOutOfNodes = errors.New("query ran out of nodes during search")

var ErrInProgress = errors.New("operation in progress")
var ErrPartialResult = errors.New("query did not reach the end location, returning best guess")
