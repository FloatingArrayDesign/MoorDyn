extern crate moordyn;
use std::ffi::CString;
use std::os::raw::c_uint;

fn main() {
    let mut err: i32;

    let fpath = CString::new("Mooring/lines.txt").expect("CString::new failed");
    unsafe {
        let system = moordyn::MoorDyn_Create(fpath.as_ptr());

        let mut n_dof: u32 = 0;
        err = moordyn::MoorDyn_NCoupledDOF(system, &mut n_dof as *mut c_uint);
        assert_eq!(err, moordyn::MOORDYN_SUCCESS as i32);
        assert_eq!(n_dof, 9);

        let mut x = vec![0.0; 9];
        let mut dx = vec![0.0; 9];
        for i in 1..=3 {
            let point = moordyn::MoorDyn_GetPoint(system, i);
            let mut pos = vec![0.0; 3];
            err = moordyn::MoorDyn_GetPointPos(point, pos.as_mut_ptr());
            assert_eq!(err, moordyn::MOORDYN_SUCCESS as i32);
            let slc : usize = (i - 1) as usize;
            x[slc..slc+3].clone_from_slice(&pos);
        }

        err = moordyn::MoorDyn_Init(system, x.as_ptr(), dx.as_ptr());
        assert_eq!(err, moordyn::MOORDYN_SUCCESS as i32);

        let mut f = Vec::<f64>::with_capacity(9);
        let mut t: f64 = 0.0;
        let mut dt: f64 = 0.5;
        err = moordyn::MoorDyn_Step(system, x.as_ptr(), dx.as_ptr(), f.as_mut_ptr(), &mut t as *mut f64, &mut dt as *mut f64);
        assert_eq!(err, moordyn::MOORDYN_SUCCESS as i32);

        err = moordyn::MoorDyn_Close(system);
        assert_eq!(err, moordyn::MOORDYN_SUCCESS as i32);
    }
}
