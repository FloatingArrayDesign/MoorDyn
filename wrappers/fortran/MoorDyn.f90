module moordyn

  use, intrinsic :: iso_fortran_env, only: int64, real32, real64, &
                                           stderr => error_unit
  use, intrinsic :: iso_c_binding, only: c_char, c_null_char, c_int, c_double, &
                                         c_ptr, c_loc

  implicit none

  ! The error codes, see MoorDynAPI.h
  integer, parameter :: MD_SUCESS = 0
  integer, parameter :: MD_INVALID_INPUT_FILE = -1
  integer, parameter :: MD_INVALID_OUTPUT_FILE = -2
  integer, parameter :: MD_INVALID_INPUT = -3
  integer, parameter :: MD_NAN_ERROR = -4
  integer, parameter :: MD_MEM_ERROR = -5
  integer, parameter :: MD_INVALID_VALUE = -6
  integer, parameter :: MD_NON_IMPLEMENTED = -7
  integer, parameter :: MD_UNHANDLED_ERROR = -255

  ! The log levels
  integer, parameter :: MD_ERR_LEVEL = 3
  integer, parameter :: MD_WRN_LEVEL = 2
  integer, parameter :: MD_MSG_LEVEL = 1
  integer, parameter :: MD_DBG_LEVEL = 0
  integer, parameter :: MD_NO_OUTPUT = 4096

  private :: MoorDyn_Create, MoorDyn_SetLogFile, MoorDyn_Log, MoorDyn_Init, &
             MoorDyn_Step, MoorDyn_ExternalWaveKinGetCoordinates, &
             MoorDyn_ExternalWaveKinSet, MoorDyn_GetFASTtens, &
             MoorDyn_GetBodyState, &
             MoorDyn_GetConnectPos, MoorDyn_GetConnectVel, &
             MoorDyn_GetConnectForce

  public :: MD_Create, MD_NCoupledDOF, MD_SetVerbosity, MD_SetLogFile, &
            MD_SetLogLevel, MD_Log, MD_Init, MD_Step, MD_Close, &
            MD_GetWaves, MD_ExternalWaveKinInit, MD_ExternalWaveKinGetN, &
            MD_ExternalWaveKinGetCoordinates, MD_ExternalWaveKinSet, &
            MD_GetNumberBodies, MD_GetBody, MD_GetNumberRods, MD_GetRod, &
            MD_GetNumberConnections, MD_GetConnection, MD_GetNumberLines, &
            MD_GetLine, MD_GetFASTtens, &
            MD_GetBodyID, MD_GetBodyType, MD_GetBodyState, &
            MD_GetConnectID, MD_GetConnectType, MD_GetConnectPos, &
            MD_GetConnectVel, MD_GetConnectForce

  interface

    !                                MoorDyn2.h
    ! ==========================================================================

    function MoorDyn_Create(f) bind(c, name='MoorDyn_Create') result(rc)
      import :: c_char, c_ptr
      character(kind=c_char), intent(in) :: f(*)
      type(c_ptr) :: rc
    end function MoorDyn_Create

    integer(c_int) function MD_NCoupledDOF(instance, n) bind(c, name='MoorDyn_NCoupledDOF')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_NCoupledDOF

    integer(c_int) function MD_SetVerbosity(instance, n) bind(c, name='MD_SetVerbosity')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_SetVerbosity

    function MoorDyn_SetLogFile(instance, f) bind(c, name='MoorDyn_SetLogFile') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SetLogFile

    integer(c_int) function MD_SetLogLevel(instance, n) bind(c, name='MoorDyn_SetLogLevel')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_SetLogLevel

    function MoorDyn_Log(instance, n, f) bind(c, name='MoorDyn_Log') result(rc)
      import :: c_ptr, c_int, c_char
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(in) :: n
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_Log

    function MoorDyn_Init(instance, x, xd) bind(c, name='MoorDyn_Init') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: x
      type(c_ptr), value, intent(in) :: xd
      integer(c_int) :: rc
    end function MoorDyn_Init

    function MoorDyn_Step(instance, x, xd, f, t, dt) bind(c, name='MoorDyn_Step') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: x
      type(c_ptr), value, intent(in) :: xd
      type(c_ptr), value, intent(in) :: f
      real(c_double), intent(inout) :: t
      real(c_double), intent(inout) :: dt
      integer(c_int) :: rc
    end function MoorDyn_Step

    integer(c_int) function MD_Close(instance) bind(c, name='MoorDyn_Close')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
    end function MD_Close

    type(c_ptr) function MD_GetWaves(instance) bind(c, name='MoorDyn_GetWaves')
      import :: c_ptr
      type(c_ptr), value, intent(in) :: instance
    end function MD_GetWaves

    integer(c_int) function MD_ExternalWaveKinInit(instance, n) bind(c, name='MoorDyn_ExternalWaveKinInit')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_ExternalWaveKinInit

    integer(c_int) function MD_ExternalWaveKinGetN(instance, n) bind(c, name='MoorDyn_ExternalWaveKinGetN')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_ExternalWaveKinGetN

    function MoorDyn_ExternalWaveKinGetCoordinates(instance, r) bind(c, name='MoorDyn_ExternalWaveKinGetCoordinates') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_ExternalWaveKinGetCoordinates

    function MoorDyn_ExternalWaveKinSet(instance, u, ud, t) bind(c, name='MoorDyn_ExternalWaveKinSet') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: u
      type(c_ptr), value, intent(in) :: ud
      real(c_double), intent(in) :: t
      integer(c_int) :: rc
    end function MoorDyn_ExternalWaveKinSet

    integer(c_int) function MD_GetNumberBodies(instance, n) bind(c, name='MoorDyn_GetNumberBodies')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberBodies

    type(c_ptr) function MD_GetBody(instance, n) bind(c, name='MoorDyn_GetBody')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(in) :: n
    end function MD_GetBody

    integer(c_int) function MD_GetNumberRods(instance, n) bind(c, name='MoorDyn_GetNumberRods')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberRods

    type(c_ptr) function MD_GetRod(instance, n) bind(c, name='MoorDyn_GetRod')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(in) :: n
    end function MD_GetRod

    integer(c_int) function MD_GetNumberConnections(instance, n) bind(c, name='MoorDyn_GetNumberConnections')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberConnections

    type(c_ptr) function MD_GetConnection(instance, n) bind(c, name='MoorDyn_GetConnection')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
    end function MD_GetConnection

    integer(c_int) function MD_GetNumberLines(instance, n) bind(c, name='MoorDyn_GetNumberLines')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberLines

    type(c_ptr) function MD_GetLine(instance, n) bind(c, name='MoorDyn_GetLine')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(in) :: n
    end function MD_GetLine

    function MoorDyn_GetFASTtens(instance, n, fht, fvt, aht, avt) bind(c, name='MoorDyn_GetFASTtens') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(inout) :: n
      type(c_ptr), value, intent(in) :: fht
      type(c_ptr), value, intent(in) :: fvt
      type(c_ptr), value, intent(in) :: aht
      type(c_ptr), value, intent(in) :: avt
      integer(c_int) :: rc
    end function MoorDyn_GetFASTtens

    !                                Body.h
    ! ==========================================================================

    integer(c_int) function MD_GetBodyID(instance, n) bind(c, name='MoorDyn_GetBodyID')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetBodyID

    integer(c_int) function MD_GetBodyType(instance, n) bind(c, name='MoorDyn_GetBodyType')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetBodyType

    function MoorDyn_GetBodyState(instance, r, rd) bind(c, name='MoorDyn_GetBodyState') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      type(c_ptr), value, intent(in) :: rd
      integer(c_int) :: rc
    end function MoorDyn_GetBodyState

    !                                Connection.h
    ! ==========================================================================

    integer(c_int) function MD_GetConnectID(instance, n) bind(c, name='MoorDyn_GetConnectID')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetConnectID

    integer(c_int) function MD_GetConnectType(instance, n) bind(c, name='MoorDyn_GetConnectType')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetConnectType

    function MoorDyn_GetConnectPos(instance, r) bind(c, name='MoorDyn_GetConnectPos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetConnectPos

    function MoorDyn_GetConnectVel(instance, r) bind(c, name='MoorDyn_GetConnectVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetConnectVel

    function MoorDyn_GetConnectForce(instance, r) bind(c, name='MoorDyn_GetConnectForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetConnectForce

end interface


contains

  !                                MoorDyn2.h
  ! ============================================================================

  type(c_ptr) function MD_Create(f)
    use iso_c_binding
    character(*), intent(in) :: f
    MD_Create = MoorDyn_Create(trim(f) // c_null_char)
  end function MD_Create

  integer function MD_SetLogFile(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SetLogFile = MoorDyn_SetLogFile(instance, trim(f) // c_null_char)
  end function MD_SetLogFile

  integer function MD_Log(instance, n, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), intent(in) :: n
    character(*), intent(in) :: f
    MD_Log = MoorDyn_Log(instance, n, trim(f) // c_null_char)
  end function MD_Log

  integer function MD_Init(instance, x, xd)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: x(:)
    real(c_double), intent(in), target :: xd(:)
    MD_Init = MoorDyn_Init(instance, c_loc(x), c_loc(xd))
  end function MD_Init

  integer function MD_Step(instance, x, xd, f, t, dt)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: x(:)
    real(c_double), intent(in), target :: xd(:)
    real(c_double), intent(in), target :: f(:)
    real(c_double), intent(inout) :: t
    real(c_double), intent(inout) :: dt
    MD_Step = MoorDyn_Step(instance, c_loc(x), c_loc(xd), c_loc(f), t, dt)
  end function MD_Step

  integer function MD_ExternalWaveKinGetCoordinates(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_ExternalWaveKinGetCoordinates = MoorDyn_ExternalWaveKinGetCoordinates(instance, c_loc(r))
  end function MD_ExternalWaveKinGetCoordinates

  integer function MD_ExternalWaveKinSet(instance, u, ud, t)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: u(:)
    real(c_double), intent(in), target :: ud(:)
    real(c_double), intent(in) :: t
    MD_ExternalWaveKinSet = MoorDyn_ExternalWaveKinSet(instance, c_loc(u), c_loc(ud), t)
  end function MD_ExternalWaveKinSet

  integer function MD_GetFASTtens(instance, n, fht, fvt, aht, avt)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), intent(inout) :: n
    real(c_double), intent(in), target :: fht(:)
    real(c_double), intent(in), target :: fvt(:)
    real(c_double), intent(in), target :: aht(:)
    real(c_double), intent(in), target :: avt(:)
    MD_GetFASTtens = MoorDyn_GetFASTtens(instance, n, c_loc(fht), c_loc(fvt), c_loc(aht), c_loc(avt))
  end function MD_GetFASTtens

  !                                Body.h
  ! ============================================================================

  integer function MD_GetBodyState(instance, r, rd)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    real(c_double), intent(in), target :: rd(:)
    MD_GetBodyState = MoorDyn_GetBodyState(instance, c_loc(r), c_loc(rd))
  end function MD_GetBodyState

  !                                Connection.h
  ! ============================================================================

  integer function MD_GetConnectPos(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetConnectPos = MoorDyn_GetConnectPos(instance, c_loc(r))
  end function MD_GetConnectPos

  integer function MD_GetConnectVel(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetConnectVel = MoorDyn_GetConnectVel(instance, c_loc(r))
  end function MD_GetConnectVel

  integer function MD_GetConnectForce(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetConnectForce = MoorDyn_GetConnectForce(instance, c_loc(r))
  end function MD_GetConnectForce

end module moordyn
