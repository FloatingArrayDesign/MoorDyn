module moordyn

  use, intrinsic :: iso_fortran_env, only: int64, real32, real64, &
                                           stderr => error_unit
  use, intrinsic :: iso_c_binding, only: c_char, c_null_char, c_int, c_double, &
                                         c_ptr, c_loc

  implicit none

  private

  public :: MD_Create, MD_Close, MD_Init, MD_Step, MD_End

  interface

    function MoorDyn_Create(f) bind(c, name='MoorDyn_Create') result(rc)
      import :: c_char, c_ptr
      character(kind=c_char), intent(in) :: f(*)
      type(c_ptr) :: rc
    end function MoorDyn_Create

    function MoorDyn_Close(instance) bind(c, name='MoorDyn_Close') result(rc)
      import :: c_ptr, c_int
      type(c_ptr), value, intent(in) :: instance
      integer(c_int) :: rc
    end function MoorDyn_Close

    function MoorDynInit(x, xd, f) bind(c, name='MoorDynInit') result(rc)
      import :: c_char, c_ptr, c_int
      character(kind=c_char), intent(in) :: f(*)
      type(c_ptr), value, intent(in) :: x
      type(c_ptr), value, intent(in) :: xd
      integer(c_int) :: rc
    end function MoorDynInit

    function MoorDynStep(x, xd, f, t, dt) bind(c, name='MoorDynStep') result(rc)
      import :: c_ptr, c_double, c_int
      type(c_ptr), value, intent(in) :: x
      type(c_ptr), value, intent(in) :: xd
      type(c_ptr), value, intent(in) :: f
      real(c_double), intent(inout) :: t
      real(c_double), intent(inout) :: dt
      integer(c_int) :: rc
    end function MoorDynStep

    function MoorDynClose() bind(c, name='MoorDynClose') result(rc)
      import :: c_int
      integer(c_int) :: rc
    end function MoorDynClose

  end interface


contains

  type(c_ptr) function MD_Create(f)
    use iso_c_binding
    character(*), intent(in) :: f
    MD_Create = MoorDyn_Create(trim(f) // c_null_char)
  end function MD_Create

  integer function MD_Close(instance)
    use iso_c_binding
    type(c_ptr), intent(in) :: instance
    MD_Close = MoorDyn_Close(instance)
  end function MD_Close

  integer function MD_Init(x, xd, f)
    use iso_c_binding
    real(c_double), intent(in), target :: x(:)
    real(c_double), intent(in), target :: xd(:)
    character(*), intent(in) :: f
    MD_Init = MoorDynInit(c_loc(x), c_loc(xd), trim(f) // c_null_char)
  end function MD_Init

  integer function MD_Step(x, xd, f, t, dt)
    use iso_c_binding
    real(c_double), intent(in), target :: x(:)
    real(c_double), intent(in), target :: xd(:)
    real(c_double), intent(in), target :: f(:)
    real(c_double), intent(inout) :: t
    real(c_double), intent(inout) :: dt
    MD_Step = MoorDynStep(c_loc(x), c_loc(xd), c_loc(f), t, dt)
  end function MD_Step

  integer function MD_End()
    use iso_c_binding
    MD_End = MoorDynClose()
  end function MD_End

end module moordyn
