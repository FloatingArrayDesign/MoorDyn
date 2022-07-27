program main
    use, intrinsic :: iso_fortran_env, only: real64
    use moordyn

    character(len=28) :: infile
    real(real64), allocatable, target :: x(:)
    real(real64), allocatable, target :: xd(:)
    real(real64), allocatable, target :: f(:)
    real(real64) :: t, dt
    integer :: err

    infile = 'Mooring/lines.txt'
    allocate ( x(0:8) )
    allocate ( xd(0:8) )
    allocate ( f(0:8) )

    x(0) = 5.2
    x(1) = 0.0
    x(2) = -70.0

    x(3) = -2.6
    x(4) = 4.5
    x(5) = -70.0

    x(6) = -2.6
    x(7) = -4.5
    x(8) = -70.0

    err = MD_Init(x, xd, infile)
    if ( err /= 0 ) then
      print *,"Failure during the mooring initialization: ", err
      stop 1
    end if

    t = 0
    dt = 0.5
    err = MD_Step(x, xd, f, t, dt)
    if ( err /= 0 ) then
      print *,"Failure during the mooring step: ", err
      stop 1
    end if

    err = MD_End()
    if ( err /= 0 ) then
      print *,"Failure closing Moordyn: ", err
      stop 1
    end if

end program main
