/*
 * Copyright (c) 2019 Matt Hall <mtjhall@alumni.uvic.ca>
 *
 * This file is part of MoorDyn.  MoorDyn is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * MoorDyn is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS

 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MoorDyn.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Instance.hpp"
#include <atomic>

namespace moordyn {

std::atomic<size_t> __instances_counter(0);

Instance::Instance(moordyn::Log* log)
  : io::IO(log)
{
	_id = __instances_counter++;
}

void
reset_instance_ids()
{
	__instances_counter = 0;
}

} // ::moordyn
