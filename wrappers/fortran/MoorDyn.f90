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
             MoorDyn_Init_NoIC, MoorDyn_Step, &
             MoorDyn_ExternalWaveKinGetCoordinates, &
             MoorDyn_ExternalWaveKinSet, MoorDyn_GetFASTtens, MoorDyn_Save, &
             MoorDyn_Load, MoorDyn_SaveVTK, &
             MoorDyn_GetWavesKin, &
             MoorDyn_GetBodyState, MoorDyn_SaveBodyVTK, MoorDyn_UseBodyVTK, &
             MoorDyn_GetPointPos, MoorDyn_GetPointVel, &
             MoorDyn_GetPointForce, MoorDyn_SavePointVTK, &
             MoorDyn_GetRodNodePos, MoorDyn_SaveRodVTK, &
             MoorDyn_GetLineNodePos, MoorDyn_GetLineNodeTen, &
             MoorDyn_SaveLineVTK

  public :: MD_Create, MD_NCoupledDOF, MD_SetVerbosity, MD_SetLogFile, &
            MD_SetLogLevel, MD_Log, MD_Init, MD_Init_NoIC, MD_Step, MD_Close, &
            MD_GetWaves, MD_ExternalWaveKinInit, MD_ExternalWaveKinGetN, &
            MD_ExternalWaveKinGetCoordinates, MD_ExternalWaveKinSet, &
            MD_GetNumberBodies, MD_GetBody, MD_GetNumberRods, MD_GetRod, &
            MD_GetNumberPoints, MD_GetPoint, MD_GetNumberLines, &
            MD_GetLine, MD_GetFASTtens, MD_Save, MD_Load, MD_SaveVTK, &
            MD_GetWavesKin, &
            MD_GetBodyID, MD_GetBodyType, MD_GetBodyState, MD_SaveBodyVTK, &
            MD_UseBodyVTK, &
            MD_GetPointID, MD_GetPointType, MD_GetPointPos, &
            MD_GetPointVel, MD_GetPointForce, MD_GetPointNAttached, &
            MD_GetPointAttached, MD_SavePointVTK, &
            MD_GetRodID, MD_GetRodType, MD_GetRodN, MD_GetRodNumberNodes, &
            MD_GetRodNodePos, MD_SaveRodVTK, &
            MD_GetLineID, MD_GetLineN, MD_GetLineNumberNodes, &
            MD_GetLineUnstretchedLength, MD_GetLineNodePos, &
            MD_GetLineNodeTen, MD_GetLineNodeCurv, MD_GetLineMaxTen, &
            MD_GetLineFairTen, MD_SaveLineVTK

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

    function MoorDyn_Init_NoIC(instance, x, xd) bind(c, name='MoorDyn_Init_NoIC') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: x
      type(c_ptr), value, intent(in) :: xd
      integer(c_int) :: rc
    end function MoorDyn_Init_NoIC

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
      integer(c_int), value, intent(in) :: n
    end function MD_GetBody

    integer(c_int) function MD_GetNumberRods(instance, n) bind(c, name='MoorDyn_GetNumberRods')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberRods

    type(c_ptr) function MD_GetRod(instance, n) bind(c, name='MoorDyn_GetRod')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
    end function MD_GetRod

    integer(c_int) function MD_GetNumberPoints(instance, n) bind(c, name='MoorDyn_GetNumberPoints')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberPoints

    type(c_ptr) function MD_GetPoint(instance, n) bind(c, name='MoorDyn_GetPoint')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
    end function MD_GetPoint

    integer(c_int) function MD_GetNumberLines(instance, n) bind(c, name='MoorDyn_GetNumberLines')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetNumberLines

    type(c_ptr) function MD_GetLine(instance, n) bind(c, name='MoorDyn_GetLine')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
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

    function MoorDyn_Save(instance, f) bind(c, name='MoorDyn_Save') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_Save

    function MoorDyn_Load(instance, f) bind(c, name='MoorDyn_Load') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_Load

    function MoorDyn_SaveVTK(instance, f) bind(c, name='MoorDyn_SaveVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveVTK

    !                                Waves.h
    ! ==========================================================================

    function MoorDyn_GetWavesKin(instance, x, y, z, t, u, ud, zeta, pdyn) bind(c, name='MoorDyn_GetWavesKin') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: x
      real(c_double), value, intent(in) :: y
      real(c_double), value, intent(in) :: z
      real(c_double), value, intent(in) :: t
      type(c_ptr), value, intent(in) :: u
      type(c_ptr), value, intent(in) :: ud
      real(c_double), intent(out) :: zeta
      real(c_double), intent(out) :: pdyn
      integer(c_int) :: rc
    end function MoorDyn_GetWavesKin

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

    function MoorDyn_SaveBodyVTK(instance, f) bind(c, name='MoorDyn_SaveBodyVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveBodyVTK

    function MoorDyn_UseBodyVTK(instance, f) bind(c, name='MoorDyn_UseBodyVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_UseBodyVTK

    !                                Rod.h
    ! ==========================================================================

    integer(c_int) function MD_GetRodID(instance, n) bind(c, name='MoorDyn_GetRodID')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetRodID

    integer(c_int) function MD_GetRodType(instance, n) bind(c, name='MoorDyn_GetRodType')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetRodType

    integer(c_int) function MD_GetRodN(instance, n) bind(c, name='MoorDyn_GetRodN')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetRodN

    integer(c_int) function MD_GetRodNumberNodes(instance, n) bind(c, name='MoorDyn_GetRodNumberNodes')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetRodNumberNodes

    function MoorDyn_GetRodNodePos(instance, n, r) bind(c, name='MoorDyn_GetRodNodePos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetRodNodePos

    function MoorDyn_SaveRodVTK(instance, f) bind(c, name='MoorDyn_SaveRodVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveRodVTK

    !                                Point.h
    ! ==========================================================================

    integer(c_int) function MD_GetPointID(instance, n) bind(c, name='MoorDyn_GetPointID')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetPointID

    integer(c_int) function MD_GetPointType(instance, n) bind(c, name='MoorDyn_GetPointType')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetPointType

    function MoorDyn_GetPointPos(instance, r) bind(c, name='MoorDyn_GetPointPos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetPointPos

    function MoorDyn_GetPointVel(instance, r) bind(c, name='MoorDyn_GetPointVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetPointVel

    function MoorDyn_GetPointForce(instance, r) bind(c, name='MoorDyn_GetPointForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetPointForce

    integer(c_int) function MD_GetPointNAttached(instance, n) bind(c, name='MoorDyn_GetPointNAttached')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetPointNAttached

    integer(c_int) function MD_GetPointAttached(instance, i, l, e) bind(c, name='MoorDyn_GetPointAttached')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: i
      type(c_ptr), intent(out) :: l
      integer(c_int), intent(out) :: e
    end function MD_GetPointAttached

    function MoorDyn_SavePointVTK(instance, f) bind(c, name='MoorDyn_SavePointVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SavePointVTK

    !                                Line.h
    ! ==========================================================================

    integer(c_int) function MD_GetLineID(instance, n) bind(c, name='MoorDyn_GetLineID')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetLineID

    integer(c_int) function MD_GetLineN(instance, n) bind(c, name='MoorDyn_GetLineN')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetLineN

    integer(c_int) function MD_GetLineNumberNodes(instance, n) bind(c, name='MoorDyn_GetLineNumberNodes')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
    end function MD_GetLineNumberNodes

    integer(c_int) function MD_GetLineUnstretchedLength(instance, l) bind(c, name='MoorDyn_GetLineUnstretchedLength')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: l
    end function MD_GetLineUnstretchedLength

    function MoorDyn_GetLineNodePos(instance, n, r) bind(c, name='MoorDyn_GetLineNodePos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodePos

    function MoorDyn_GetLineNodeTen(instance, n, r) bind(c, name='MoorDyn_GetLineNodeTen') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeTen

    integer(c_int) function MD_GetLineNodeCurv(instance, n, r) bind(c, name='MoorDyn_GetLineNodeCurv') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      real(c_double), intent(out) :: r
    end function MD_GetLineNodeCurv

    integer(c_int) function MD_GetLineFairTen(instance, r) bind(c, name='MoorDyn_GetLineFairTen') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: r
    end function MD_GetLineFairTen

    integer(c_int) function MD_GetLineMaxTen(instance, r) bind(c, name='MoorDyn_GetLineMaxTen') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: r
    end function MD_GetLineMaxTen

    function MoorDyn_SaveLineVTK(instance, f) bind(c, name='MoorDyn_SaveLineVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveLineVTK
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

  integer function MD_Init_NoIC(instance, x, xd)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: x(:)
    real(c_double), intent(in), target :: xd(:)
    MD_Init_NoIC = MoorDyn_Init_NoIC(instance, c_loc(x), c_loc(xd))
  end function MD_Init_NoIC

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

  integer function MD_Save(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_Save = MoorDyn_Save(instance, trim(f) // c_null_char)
  end function MD_Save

  integer function MD_Load(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_Load = MoorDyn_Load(instance, trim(f) // c_null_char)
  end function MD_Load

  integer function MD_SaveVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveVTK = MoorDyn_SaveVTK(instance, trim(f) // c_null_char)
  end function MD_SaveVTK

  !                                Waves.h
  ! ============================================================================

  integer function MD_GetWavesKin(instance, x, y, z, t, u, ud, zeta, pdyn)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), value, intent(in) :: x
    real(c_double), value, intent(in) :: y
    real(c_double), value, intent(in) :: z
    real(c_double), value, intent(in) :: t
    real(c_double), intent(in), target :: u(:)
    real(c_double), intent(in), target :: ud(:)
    real(c_double), intent(out) :: zeta
    real(c_double), intent(out) :: pdyn
    MD_GetWavesKin = MoorDyn_GetWavesKin(instance, x, y, z, t, c_loc(u), c_loc(ud), zeta, pdyn)
  end function MD_GetWavesKin

  !                                Body.h
  ! ============================================================================

  integer function MD_GetBodyState(instance, r, rd)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    real(c_double), intent(in), target :: rd(:)
    MD_GetBodyState = MoorDyn_GetBodyState(instance, c_loc(r), c_loc(rd))
  end function MD_GetBodyState

  integer function MD_SaveBodyVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveBodyVTK = MoorDyn_SaveBodyVTK(instance, trim(f) // c_null_char)
  end function MD_SaveBodyVTK

  integer function MD_UseBodyVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_UseBodyVTK = MoorDyn_UseBodyVTK(instance, trim(f) // c_null_char)
  end function MD_UseBodyVTK

  !                                Rod.h
  ! ============================================================================

  integer function MD_GetRodNodePos(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetRodNodePos = MoorDyn_GetRodNodePos(instance, n, c_loc(r))
  end function MD_GetRodNodePos

  integer function MD_SaveRodVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveRodVTK = MoorDyn_SaveRodVTK(instance, trim(f) // c_null_char)
  end function MD_SaveRodVTK

  !                                Point.h
  ! ============================================================================

  integer function MD_GetPointPos(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetPointPos = MoorDyn_GetPointPos(instance, c_loc(r))
  end function MD_GetPointPos

  integer function MD_GetPointVel(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetPointVel = MoorDyn_GetPointVel(instance, c_loc(r))
  end function MD_GetPointVel

  integer function MD_GetPointForce(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetPointForce = MoorDyn_GetPointForce(instance, c_loc(r))
  end function MD_GetPointForce

  integer function MD_SavePointVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SavePointVTK = MoorDyn_SavePointVTK(instance, trim(f) // c_null_char)
  end function MD_SavePointVTK

  !                                Line.h
  ! ============================================================================

  integer function MD_GetLineNodePos(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodePos = MoorDyn_GetLineNodePos(instance, n, c_loc(r))
  end function MD_GetLineNodePos

  integer function MD_GetLineNodeTen(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeTen = MoorDyn_GetLineNodeTen(instance, n, c_loc(r))
  end function MD_GetLineNodeTen

  integer function MD_SaveLineVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveLineVTK = MoorDyn_SaveLineVTK(instance, trim(f) // c_null_char)
  end function MD_SaveLineVTK

end module moordyn
