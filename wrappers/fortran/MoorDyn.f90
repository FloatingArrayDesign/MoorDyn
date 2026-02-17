module moordyn

  use, intrinsic :: iso_fortran_env, only: int64, real32, real64, &
                                           stderr => error_unit
  use, intrinsic :: iso_c_binding, only: c_char, c_null_char, c_int, c_double, &
                                         c_ptr, c_loc, c_size_t, c_int64_t

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
             MoorDyn_ExternalWaveKinSet, MoorDyn_GetFASTtens, &
             MoorDyn_GetDt, MoorDyn_GetCFL, &
             MoorDyn_GetTimeScheme, MoorDyn_SetTimeScheme, &
             MoorDyn_SaveState, MoorDyn_LoadState, &
             MoorDyn_Serialize, MoorDyn_Deserialize, &
             MoorDyn_Save, MoorDyn_Load, MoorDyn_SaveVTK, &
             MoorDyn_GetWavesKin, &
             MoorDyn_GetBodyState, MoorDyn_GetBodyPos, MoorDyn_GetBodyAngle, &
             MoorDyn_GetBodyVel, MoorDyn_GetBodyAngVel, MoorDyn_GetBodyForce, &
             MoorDyn_GetBodyM, MoorDyn_SaveBodyVTK, &
             MoorDyn_GetRodForce, MoorDyn_GetRodM, MoorDyn_GetRodNodePos, &
             MoorDyn_GetRodNodeVel, MoorDyn_SaveRodVTK, &
             MoorDyn_GetPointPos, MoorDyn_GetPointVel, &
             MoorDyn_GetPointForce, MoorDyn_GetPointM, MoorDyn_SavePointVTK, &
             MoorDyn_SetLinePressInt, MoorDyn_GetLineNodePos, &
             MoorDyn_GetLineNodeVel, MoorDyn_GetLineNodeForce, &
             MoorDyn_GetLineNodeTen, MoorDyn_GetLineNodeBendStiff, &
             MoorDyn_GetLineNodeWeight, MoorDyn_GetLineNodeDrag, &
             MoorDyn_GetLineNodeFroudeKrilov, MoorDyn_GetLineNodeSeabedForce, &
             MoorDyn_GetLineNodeM, MoorDyn_SaveLineVTK

  public :: MD_Create, MD_NCoupledDOF, MD_SetVerbosity, MD_SetLogFile, &
            MD_SetLogLevel, MD_Log, MD_Init, MD_Init_NoIC, MD_Step, MD_Close, &
            MD_GetWaves, MD_GetSeafloor, MD_ExternalWaveKinInit, &
            MD_ExternalWaveKinGetN, MD_ExternalWaveKinGetCoordinates, &
            MD_ExternalWaveKinSet, MD_GetNumberBodies, MD_GetBody, &
            MD_GetNumberRods, MD_GetRod, MD_GetNumberPoints, MD_GetPoint, &
            MD_GetNumberLines, MD_GetLine, MD_GetFASTtens, &
            MD_GetDt, MD_SetDt, MD_GetCFL, MD_SetCFL, &
            MD_GetTimeScheme, MD_SetTimeScheme, &
            MD_SaveState, MD_LoadState, MD_Serialize, MD_Deserialize, &
            MD_Save, MD_Load, MD_SaveVTK, &
            MD_GetWavesKin, &
            MD_GetDepthAt, MD_GetAverageDepth, MD_GetMinDepth, &
            MD_GetBodyID, MD_GetBodyType, MD_GetBodyState, MD_GetBodyPos, &
            MD_GetBodyAngle, MD_GetBodyVel, MD_GetBodyAngVel, &
            MD_GetBodyForce, MD_GetBodyM, MD_SaveBodyVTK, &
            MD_GetRodID, MD_GetRodType, MD_GetRodForce, MD_GetRodM, &
            MD_GetRodN, MD_GetRodNumberNodes, MD_GetRodNodePos, &
            MD_GetRodNodeVel, MD_SaveRodVTK, &
            MD_GetPointID, MD_GetPointType, MD_GetPointPos, &
            MD_GetPointVel, MD_GetPointForce, MD_GetPointM, &
            MD_GetPointNAttached, MD_GetPointAttached, MD_SavePointVTK, &
            MD_GetLineID, MD_GetLineN, MD_GetLineNumberNodes, &
            MD_GetLineUnstretchedLength, MD_SetLineUnstretchedLength, &
            MD_SetLineUnstretchedLengthVel, MD_IsLineConstantEA, &
            MD_GetLineConstantEA, MD_SetLineConstantEA, &
            MD_IsLinePressBend, MD_SetLinePressBend, MD_SetLinePressInt, &
            MD_GetLineNodePos, MD_GetLineNodeVel, MD_GetLineNodeForce, &
            MD_GetLineNodeTen, MD_GetLineNodeBendStiff, MD_GetLineNodeWeight, &
            MD_GetLineNodeDrag, MD_GetLineNodeFroudeKrilov, &
            MD_GetLineNodeSeabedForce, MD_GetLineNodeCurv, &
            MD_GetLineNodeM, MD_GetLineMaxTen, MD_GetLineFairTen, &
            MD_SaveLineVTK

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
      integer(c_int), intent(in) :: n
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
      integer(c_int), intent(in) :: n
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

    type(c_ptr) function MD_GetSeafloor(instance) bind(c, name='MoorDyn_GetSeafloor')
      import :: c_ptr
      type(c_ptr), value, intent(in) :: instance
    end function MD_GetSeafloor

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

    function MoorDyn_GetDt(instance, dt) bind(c, name='MoorDyn_GetDt') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: dt
      integer(c_int) :: rc
    end function MoorDyn_GetDt

    integer(c_int) function MD_SetDt(instance, dt) bind(c, name='MoorDyn_SetDt')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: dt
    end function MD_SetDt

    function MoorDyn_GetCFL(instance, cfl) bind(c, name='MoorDyn_GetCFL') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: cfl
      integer(c_int) :: rc
    end function MoorDyn_GetCFL

    integer(c_int) function MD_SetCFL(instance, cfl) bind(c, name='MoorDyn_SetCFL')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: cfl
    end function MD_SetCFL

    function MoorDyn_GetTimeScheme(instance, name, name_len) bind(c, name='MoorDyn_GetTimeScheme') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(out) :: name(*)
      integer(c_int), intent(inout) :: name_len
      integer(c_int) :: rc
    end function MoorDyn_GetTimeScheme

    function MoorDyn_SetTimeScheme(instance, name) bind(c, name='MoorDyn_SetTimeScheme') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: name(*)
      integer(c_int) :: rc
    end function MoorDyn_SetTimeScheme

    function MoorDyn_SaveState(instance, f) bind(c, name='MoorDyn_SaveState') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveState

    function MoorDyn_LoadState(instance, f) bind(c, name='MoorDyn_LoadState') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_LoadState

    function MoorDyn_Serialize(instance, data_size, data) bind(c, name='MoorDyn_Serialize') result(rc)
      import :: c_ptr, c_size_t, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_size_t), intent(inout) :: data_size
      type(c_ptr), value, intent(in) :: data
      integer(c_int) :: rc
    end function MoorDyn_Serialize

    function MoorDyn_Deserialize(instance, data) bind(c, name='MoorDyn_Deserialize') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: data
      integer(c_int) :: rc
    end function MoorDyn_Deserialize

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

    function MoorDyn_GetWavesKin(instance, x, y, z, t, u, ud, zeta, pdyn, seafloor) bind(c, name='MoorDyn_GetWavesKin') result(rc)
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
      type(c_ptr), value, intent(in) :: seafloor
      integer(c_int) :: rc
    end function MoorDyn_GetWavesKin

    !                                Seafloor.h
    ! ============================================================================

    integer(c_int) function MD_GetDepthAt(instance, x, y, depth) bind(c, name='MoorDyn_GetDepthAt')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: x
      real(c_double), value, intent(in) :: y
      real(c_double), intent(out) :: depth
    end function MD_GetDepthAt

    integer(c_int) function MD_GetAverageDepth(instance, depth) bind(c, name='MoorDyn_GetAverageDepth')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: depth
    end function MD_GetAverageDepth

    integer(c_int) function MD_GetMinDepth(instance, depth) bind(c, name='MoorDyn_GetMinDepth')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: depth
    end function MD_GetMinDepth

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

    function MoorDyn_GetBodyPos(instance, r) bind(c, name='MoorDyn_GetBodyPos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetBodyPos

    function MoorDyn_GetBodyAngle(instance, r) bind(c, name='MoorDyn_GetBodyAngle') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetBodyAngle

    function MoorDyn_GetBodyVel(instance, r) bind(c, name='MoorDyn_GetBodyVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetBodyVel

    function MoorDyn_GetBodyAngVel(instance, r) bind(c, name='MoorDyn_GetBodyAngVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetBodyAngVel

    function MoorDyn_GetBodyForce(instance, f) bind(c, name='MoorDyn_GetBodyForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: f
      integer(c_int) :: rc
    end function MoorDyn_GetBodyForce

    function MoorDyn_GetBodyM(instance, m) bind(c, name='MoorDyn_GetBodyM') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: m
      integer(c_int) :: rc
    end function MoorDyn_GetBodyM

    function MoorDyn_SaveBodyVTK(instance, f) bind(c, name='MoorDyn_SaveBodyVTK') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SaveBodyVTK

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

    function MoorDyn_GetRodForce(instance, f) bind(c, name='MoorDyn_GetRodForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: f
      integer(c_int) :: rc
    end function MoorDyn_GetRodForce

    function MoorDyn_GetRodM(instance, m) bind(c, name='MoorDyn_GetRodM') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: m
      integer(c_int) :: rc
    end function MoorDyn_GetRodM

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

    function MoorDyn_GetRodNodeVel(instance, n, r) bind(c, name='MoorDyn_GetRodNodeVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetRodNodeVel

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

    function MoorDyn_GetPointM(instance, m) bind(c, name='MoorDyn_GetPointM') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: m
      integer(c_int) :: rc
    end function MoorDyn_GetPointM

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

    integer(c_int) function MD_IsLineConstantEA(instance, t) bind(c, name='MoorDyn_IsLineConstantEA')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: t
    end function MD_IsLineConstantEA

    integer(c_int) function MD_GetLineConstantEA(instance, ea) bind(c, name='MoorDyn_GetLineConstantEA')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: ea
    end function MD_GetLineConstantEA

    integer(c_int) function MD_SetLineConstantEA(instance, ea) bind(c, name='MoorDyn_SetLineConstantEA')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: ea
    end function MD_SetLineConstantEA

    integer(c_int) function MD_IsLinePressBend(instance, t) bind(c, name='MoorDyn_IsLinePressBend')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: t
    end function MD_IsLinePressBend

    integer(c_int) function MD_SetLinePressBend(instance, b) bind(c, name='MoorDyn_SetLinePressBend')
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: b
    end function MD_SetLinePressBend

    function MoorDyn_SetLinePressInt(instance, p) bind(c, name='MoorDyn_SetLinePressInt') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      type(c_ptr), value, intent(in) :: p
      integer(c_int) :: rc
    end function MoorDyn_SetLinePressInt

    integer(c_int) function MD_GetLineUnstretchedLength(instance, l) bind(c, name='MoorDyn_GetLineUnstretchedLength')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), intent(out) :: l
    end function MD_GetLineUnstretchedLength

    integer(c_int) function MD_SetLineUnstretchedLength(instance, l) bind(c, name='MoorDyn_SetLineUnstretchedLength')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: l
    end function MD_SetLineUnstretchedLength

    integer(c_int) function MD_SetLineUnstretchedLengthVel(instance, l) bind(c, name='MoorDyn_SetLineUnstretchedLengthVel')
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      real(c_double), value, intent(in) :: l
    end function MD_SetLineUnstretchedLengthVel

    function MoorDyn_GetLineNodePos(instance, n, r) bind(c, name='MoorDyn_GetLineNodePos') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodePos

    function MoorDyn_GetLineNodeVel(instance, n, r) bind(c, name='MoorDyn_GetLineNodeVel') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeVel

    function MoorDyn_GetLineNodeForce(instance, n, r) bind(c, name='MoorDyn_GetLineNodeForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeForce

    function MoorDyn_GetLineNodeTen(instance, n, r) bind(c, name='MoorDyn_GetLineNodeTen') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeTen

    function MoorDyn_GetLineNodeBendStiff(instance, n, r) bind(c, name='MoorDyn_GetLineNodeBendStiff') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeBendStiff

    function MoorDyn_GetLineNodeWeight(instance, n, r) bind(c, name='MoorDyn_GetLineNodeWeight') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeWeight

    function MoorDyn_GetLineNodeDrag(instance, n, r) bind(c, name='MoorDyn_GetLineNodeDrag') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeDrag

    function MoorDyn_GetLineNodeFroudeKrilov(instance, n, r) bind(c, name='MoorDyn_GetLineNodeFroudeKrilov') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeFroudeKrilov

    function MoorDyn_GetLineNodeSeabedForce(instance, n, r) bind(c, name='MoorDyn_GetLineNodeSeabedForce') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: r
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeSeabedForce

    integer(c_int) function MD_GetLineNodeCurv(instance, n, r) bind(c, name='MoorDyn_GetLineNodeCurv') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      real(c_double), intent(out) :: r
    end function MD_GetLineNodeCurv

    function MoorDyn_GetLineNodeM(instance, n, m) bind(c, name='MoorDyn_GetLineNodeM') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), value, intent(in) :: n
      type(c_ptr), value, intent(in) :: m
      integer(c_int) :: rc
    end function MoorDyn_GetLineNodeM

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

  integer function MD_GetDt(instance, dt)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: dt(:)
    MD_GetDt = MoorDyn_GetDt(instance, c_loc(dt))
  end function MD_GetDt

  integer function MD_GetCFL(instance, cfl)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: cfl(:)
    MD_GetCFL = MoorDyn_GetCFL(instance, c_loc(cfl))
  end function MD_GetCFL

  integer function MD_GetTimeScheme(instance, name, name_len)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(len=*), intent(out) :: name
    integer(c_int), intent(inout) :: name_len
    character(:, kind=c_char), allocatable :: cname
    integer(c_int) :: i
    allocate(character(name_len + 1) :: cname)
    MD_GetTimeScheme = MoorDyn_GetTimeScheme(instance, cname, name_len)
    name = cname(1:name_len-1)
  end function MD_GetTimeScheme

  integer function MD_SetTimeScheme(instance, name)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: name
    MD_SetTimeScheme = MoorDyn_SetTimeScheme(instance, trim(name) // c_null_char)
  end function MD_SetTimeScheme

  integer function MD_SaveState(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveState = MoorDyn_SaveState(instance, trim(f) // c_null_char)
  end function MD_SaveState

  integer function MD_LoadState(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_LoadState = MoorDyn_LoadState(instance, trim(f) // c_null_char)
  end function MD_LoadState

  integer function MD_Serialize(instance, data_size, data)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_size_t), intent(inout) :: data_size
    integer(c_int64_t), intent(in), target :: data(:)
    MD_Serialize = MoorDyn_Serialize(instance, data_size, c_loc(data))
  end function MD_Serialize

  integer function MD_Deserialize(instance, data)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int64_t), intent(in), target :: data(:)
    MD_Deserialize = MoorDyn_Deserialize(instance, c_loc(data))
  end function MD_Deserialize

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

  integer function MD_GetWavesKin(instance, x, y, z, t, u, ud, zeta, pdyn, seafloor)
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
    type(c_ptr), intent(in) :: seafloor
    MD_GetWavesKin = MoorDyn_GetWavesKin(instance, x, y, z, t, c_loc(u), c_loc(ud), zeta, pdyn, seafloor)
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

  integer function MD_GetBodyPos(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetBodyPos = MoorDyn_GetBodyPos(instance, c_loc(r))
  end function MD_GetBodyPos

  integer function MD_GetBodyAngle(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetBodyAngle = MoorDyn_GetBodyAngle(instance, c_loc(r))
  end function MD_GetBodyAngle

  integer function MD_GetBodyVel(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetBodyVel = MoorDyn_GetBodyVel(instance, c_loc(r))
  end function MD_GetBodyVel

  integer function MD_GetBodyAngVel(instance, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: r(:)
    MD_GetBodyAngVel = MoorDyn_GetBodyAngVel(instance, c_loc(r))
  end function MD_GetBodyAngVel

  integer function MD_GetBodyForce(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: f(:)
    MD_GetBodyForce = MoorDyn_GetBodyForce(instance, c_loc(f))
  end function MD_GetBodyForce

  integer function MD_GetBodyM(instance, m)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: m(:)
    MD_GetBodyM = MoorDyn_GetBodyM(instance, c_loc(m))
  end function MD_GetBodyM

  integer function MD_SaveBodyVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveBodyVTK = MoorDyn_SaveBodyVTK(instance, trim(f) // c_null_char)
  end function MD_SaveBodyVTK

  !                                Rod.h
  ! ============================================================================

  integer function MD_GetRodForce(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: f(:)
    MD_GetRodForce = MoorDyn_GetRodForce(instance, c_loc(f))
  end function MD_GetRodForce

  integer function MD_GetRodM(instance, m)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: m(:)
    MD_GetRodM = MoorDyn_GetRodM(instance, c_loc(m))
  end function MD_GetRodM

  integer function MD_GetRodNodePos(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetRodNodePos = MoorDyn_GetRodNodePos(instance, n, c_loc(r))
  end function MD_GetRodNodePos

  integer function MD_GetRodNodeVel(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetRodNodeVel = MoorDyn_GetRodNodeVel(instance, n, c_loc(r))
  end function MD_GetRodNodeVel

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

  integer function MD_GetPointM(instance, m)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: m(:)
    MD_GetPointM = MoorDyn_GetPointM(instance, c_loc(m))
  end function MD_GetPointM

  integer function MD_SavePointVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SavePointVTK = MoorDyn_SavePointVTK(instance, trim(f) // c_null_char)
  end function MD_SavePointVTK

  !                                Line.h
  ! ============================================================================

  integer function MD_SetLinePressInt(instance, p)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    real(c_double), intent(in), target :: p(:)
    MD_SetLinePressInt = MoorDyn_SetLinePressInt(instance, c_loc(p))
  end function MD_SetLinePressInt

  integer function MD_GetLineNodePos(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodePos = MoorDyn_GetLineNodePos(instance, n, c_loc(r))
  end function MD_GetLineNodePos

  integer function MD_GetLineNodeVel(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeVel = MoorDyn_GetLineNodeVel(instance, n, c_loc(r))
  end function MD_GetLineNodeVel

  integer function MD_GetLineNodeForce(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeForce = MoorDyn_GetLineNodeForce(instance, n, c_loc(r))
  end function MD_GetLineNodeForce

  integer function MD_GetLineNodeTen(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeTen = MoorDyn_GetLineNodeTen(instance, n, c_loc(r))
  end function MD_GetLineNodeTen

  integer function MD_GetLineNodeBendStiff(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeBendStiff = MoorDyn_GetLineNodeBendStiff(instance, n, c_loc(r))
  end function MD_GetLineNodeBendStiff

  integer function MD_GetLineNodeWeight(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeWeight = MoorDyn_GetLineNodeWeight(instance, n, c_loc(r))
  end function MD_GetLineNodeWeight

  integer function MD_GetLineNodeDrag(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeDrag = MoorDyn_GetLineNodeDrag(instance, n, c_loc(r))
  end function MD_GetLineNodeDrag

  integer function MD_GetLineNodeFroudeKrilov(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeFroudeKrilov = MoorDyn_GetLineNodeFroudeKrilov(instance, n, c_loc(r))
  end function MD_GetLineNodeFroudeKrilov

  integer function MD_GetLineNodeSeabedForce(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeSeabedForce = MoorDyn_GetLineNodeSeabedForce(instance, n, c_loc(r))
  end function MD_GetLineNodeSeabedForce

  integer function MD_GetLineNodeM(instance, n, r)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), value, intent(in) :: n
    real(c_double), intent(in), target :: r(:)
    MD_GetLineNodeM = MoorDyn_GetLineNodeM(instance, n, c_loc(r))
  end function MD_GetLineNodeM

  integer function MD_SaveLineVTK(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SaveLineVTK = MoorDyn_SaveLineVTK(instance, trim(f) // c_null_char)
  end function MD_SaveLineVTK

end module moordyn
