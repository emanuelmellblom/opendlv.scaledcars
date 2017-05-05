/**
 * overtaker - Sample application for overtaking obstacles.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//#ifndef OVERTAKERTESTSUITE_H_
//#define OVERTAKERTESTSUITE_H_

#ifndef overtaker_TESTSUITE_H_
#define overtaker_TESTSUITE_H_

#include "cxxtest/TestSuite.h"

// Include local header files.
#include "../include/Overtaker.h"

using namespace std;

class ExamplePerceptionTest : public CxxTest::TestSuite {

    public:
        void setUp() {
        }

        void tearDown() {
        }

        void testApplication() {
            TS_ASSERT(true);
        }
};

#endif /*OVERTAKERTESTSUITE_H_*/
 



