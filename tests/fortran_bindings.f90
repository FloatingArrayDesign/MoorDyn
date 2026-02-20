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
  integer :: err, n_dof, n_points, i_point, n_lines, i_line, n_nodes, i_node
  type(c_ptr) :: system, point, line

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
  allocate ( r(0:2) )
  xd = 0.0
  f = 0.0

  ! Get the positions from the points
  err = MD_GetNumberPoints( system, n_points )
  if ( err /= MD_SUCESS ) then
    stop 1
  elseif ( n_points /= 6 ) then
    print *,"6 points were expected, not ", n_points
  end if
  do i_point = 1, 3
    point = MD_GetPoint( system, i_point + 3 )
    if ( .not.c_associated(point) ) then
      stop 1
    end if
    err = MD_GetPointPos( point, r )
    if ( err /= MD_SUCESS ) then
      stop 1
    end if
    do j = 1, 3
      x(3 * (i_point - 1) + (j - 1)) = r(j - 1)
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

  ! Print the position and tension of the line nodes
  err = MD_GetNumberLines(system, n_lines)
  if ( err /= MD_SUCESS ) then
    stop 1
  end if
  do i_line = 1, n_lines
    print *,"Line ", i_line
    print *, "======="
    line = MD_GetLine(system, i_line)
    err = MD_GetLineNumberNodes(line, n_nodes)
    do i_node = 0, n_nodes - 1
      print *,"  node ", i_node, ":"
      err = MD_GetLineNodePos(line, i_node, r)
      print *,"  pos = ", r
      err = MD_GetLineNodeTen(line, i_node, r)
      print *,"  ten = ", r
    end do
  end do

  err = MD_Close(system)
  if ( err /= MD_SUCESS ) then
    stop 1
  end if

  deallocate ( x )
  deallocate ( xd )
  deallocate ( f )
  deallocate ( r )

end program main
