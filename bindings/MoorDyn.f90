module moordyn

  use, intrinsic :: iso_fortran_env, only: int64, real32, real64, &
                                           stderr => error_unit
  use, intrinsic :: iso_c_binding, only: c_char, c_null_char, c_int, c_double, &
                                         c_ptr, c_loc

  implicit none

  private

  public :: MD_Init, MD_Step, MD_End

  interface

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
