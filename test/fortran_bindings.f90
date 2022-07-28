program main
  use, intrinsic :: iso_fortran_env, only: real64
  use, intrinsic :: iso_c_binding, only: c_ptr, c_associated
  use moordyn

  character(len=28) :: infile
  real(real64), allocatable, target :: x(:)
  real(real64), allocatable, target :: xd(:)
  real(real64), allocatable, target :: f(:)
  real(real64), allocatable, target :: r(:)
  real(real64) :: t, dt
  integer :: err, n_dof, n_conns, i_conn
  type(c_ptr) :: system, conn

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
  xd = 0.0
  f = 0.0

  ! Get the positions from the connections
  err = MD_GetNumberConnections( system, n_conns )
  if ( err /= MD_SUCESS ) then
    stop 1
  elseif ( n_conns /= 6 ) then
    print *,"6 connections were expected, not ", n_conns
  end if
  allocate ( r(0:2) )
  do i_conn = 1, 3
    conn = MD_GetConnection( system, i_conn + 3 )
    if ( .not.c_associated(conn) ) then
      stop 1
    end if
    err = MD_GetConnectPos( conn, r )    
    if ( err /= MD_SUCESS ) then
      stop 1
    end if
    do j = 1, 3
      x(3 * i + j) = r(j)
    end do
  end do

  err = MD_Init(system, x, xd)
  if ( err /= MD_SUCESS ) then
    stop 1
  end if

  t = 0
  dt = 0.5
  err = MD_Step(system, x, xd, f, t, dt)
  if ( err /= MD_SUCESS ) then
    stop 1
  end if

  err = MD_Close(system)
  if ( err /= MD_SUCESS ) then
    stop 1
  end if

end program main
