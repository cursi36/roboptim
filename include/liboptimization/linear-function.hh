// Copyright (C) 2009 by Thomas Moulard, FIXME.
//
// This file is part of the liboptimization.
//
// liboptimization is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// liboptimization is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with liboptimization.  If not, see <http://www.gnu.org/licenses/>.

/**
 * \brief Declaration of the LinearFunction class.
 */

#ifndef OPTIMIZATION_LINEAR_FUNCTION_HH
# define OPTIMIZATION_LINEAR_FUNCTION_HH
# include <limits>
# include <liboptimization/quadratic-function.hh>

namespace optimization
{
  class LinearFunction : public QuadraticFunction
  {
  public:
    LinearFunction (size_type n, value_type infinity =
                    std::numeric_limits<value_type>::infinity ()) throw ();
  };

}; // end of namespace optimization

#endif //! OPTIMIZATION_LINEAR_FUNCTION_HH