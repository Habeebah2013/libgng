#!/usr/bin/env python

import sys
sys.path.append('..')

import os
import unittest
import vec2, vec3, vec4, mat3, mat4, quat, point_cloud, vec
modules = [vec2, vec3, vec4, mat3, mat4, quat, point_cloud, vec]

suites = []
for m in modules:
    s = unittest.TestLoader().loadTestsFromTestCase(m.TestCase)
    suites.append(s)

suite = unittest.TestSuite(suites)
unittest.TextTestRunner(verbosity=2).run(suite)

os.system('python2 ../../testsuites/cu/check-regressions regressions')
