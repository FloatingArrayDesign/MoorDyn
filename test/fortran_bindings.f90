program main
    use, intrinsic :: iso_fortran_env, only: real64
    use, intrinsic :: iso_c_binding, only: c_ptr, c_associated
    use moordyn

    character(len=28) :: infile
    real(real64), allocatable, target :: x(:)
    real(real64), allocatable, target :: xd(:)
    real(real64), allocatable, target :: f(:)
    real(real64) :: t, dt
    integer :: err, n_dof
    type(c_ptr) :: system

    infile = 'Mooring/lines.txt'

    system = MD_Create(infile)
    if ( .not.c_associated(system) ) then
      stop 1
    end if

    err = MD_NCoupledDOF( system, n_dof )
    if ( err /= MD_SUCESS ) then
      stop 1
    elseif ( n_dof /= 9 ) then
      print *,"3x3 = 9 DOFs were expected, not ", n_dof
    end if

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
    xd = 0.0
    f = 0.0

    err = MD_Init(system, x, xd)
    if ( err /= MD_SUCESS ) then
      print *,"Failure during the mooring initialization: ", err
      stop 1
    end if

    t = 0
    dt = 0.5
    err = MD_Step(system, x, xd, f, t, dt)
    if ( err /= MD_SUCESS ) then
      print *,"Failure during the mooring step: ", err
      stop 1
    end if

    err = MD_Close(system)
    if ( err /= MD_SUCESS ) then
      print *,"Failure closing the moordyn simulation: ", err
      stop 1
    end if


end program main
