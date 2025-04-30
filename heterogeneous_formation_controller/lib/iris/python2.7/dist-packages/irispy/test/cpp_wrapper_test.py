import irispy.iris_wrapper as iris_wrapper
import numpy as np

def testCppWrapper():
    p = iris_wrapper.Polyhedron()
    p.setA(np.eye(2))
    p.setB(np.array([3.0, 4.0]))

    print p.contains(np.array([2.5, 5.5]), 0.0)
    try:
        print p.contains(5, 0.0)
    except NotImplementedError as e:
        print("(successfully threw the expected error)")
    except Exception as e:
        if str(e) == "The given input is not known as a NumPy array or matrix.":
            print("(successfully threw the expected error)")
        else:
            raise e

    print p.generatorPoints()
    print p.generatorPoints()[0]
    for pt in p.generatorPoints():
        print "generator:", pt

if __name__ == '__main__':
    testCppWrapper()
