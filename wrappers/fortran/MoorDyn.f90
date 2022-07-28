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

  private

  public :: MD_Create, MD_NCoupledDOF, MD_SetVerbosity, MD_SetLogFile, &
            MD_SetLogLevel, MD_Log, MD_Init, MD_Step, MD_Close

  interface

    function MoorDyn_Create(f) bind(c, name='MoorDyn_Create') result(rc)
      import :: c_char, c_ptr
      character(kind=c_char), intent(in) :: f(*)
      type(c_ptr) :: rc
    end function MoorDyn_Create

    function MoorDyn_NCoupledDOF(instance, n) bind(c, name='MoorDyn_NCoupledDOF') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
      integer(c_int) :: rc
    end function MoorDyn_NCoupledDOF

    function MoorDyn_SetVerbosity(instance, n) bind(c, name='MoorDyn_SetVerbosity') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
      integer(c_int) :: rc
    end function MoorDyn_SetVerbosity

    function MoorDyn_SetLogFile(instance, f) bind(c, name='MoorDyn_SetLogFile') result(rc)
      import :: c_ptr, c_char, c_int
      type(c_ptr), value, intent(in) :: instance
      character(kind=c_char), intent(in) :: f(*)
      integer(c_int) :: rc
    end function MoorDyn_SetLogFile

    function MoorDyn_SetLogLevel(instance, n) bind(c, name='MoorDyn_SetLogLevel') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int), intent(out) :: n
      integer(c_int) :: rc
    end function MoorDyn_SetLogLevel

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

    function MoorDyn_Close(instance) bind(c, name='MoorDyn_Close') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int) :: rc
    end function MoorDyn_Close

  end interface


contains

  type(c_ptr) function MD_Create(f)
    use iso_c_binding
    character(*), intent(in) :: f
    MD_Create = MoorDyn_Create(trim(f) // c_null_char)
  end function MD_Create

  integer function MD_NCoupledDOF(instance, n)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), intent(out) :: n
    integer err
    MD_NCoupledDOF = MoorDyn_NCoupledDOF(instance, n)
  end function MD_NCoupledDOF

  integer function MD_SetVerbosity(instance, n)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), intent(out) :: n
    integer err
    MD_SetVerbosity = MoorDyn_SetVerbosity(instance, n)
  end function MD_SetVerbosity

  integer function MD_SetLogFile(instance, f)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    character(*), intent(in) :: f
    MD_SetLogFile = MoorDyn_SetLogFile(instance, trim(f) // c_null_char)
  end function MD_SetLogFile

  integer function MD_SetLogLevel(instance, n)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    integer(c_int), intent(out) :: n
    integer err
    MD_SetLogLevel = MoorDyn_SetLogLevel(instance, n)
  end function MD_SetLogLevel

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

  integer function MD_Close(instance)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    MD_Close = MoorDyn_Close(instance)
  end function MD_Close

end module moordyn
