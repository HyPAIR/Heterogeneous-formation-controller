import ctypes
import operator
import sys

import codecs
codecs.lookup('utf-8')

int32 = ctypes.c_int
int64 = ctypes.c_longlong
float64 = ctypes.c_double

def array(v,t=None):
    def mkfactory(v,t):
        if isinstance(v,ctypes.Array) and (t is None or v._type_ is t):
            return lambda x:x
        else:
            if   t is int32:
                return lambda x: (ctypes.c_int * len(v))(*x)
            elif t is int64:
                return lambda x: (ctypes.c_longlong * len(v))(*x)
            elif t is float64 or t is float:
                return lambda x: (ctypes.c_double * len(v))(*x)
            elif t is None:
                if len(v) == 0:
                    return mkfactory(v,int32)
                elif   isinstance(v[0],long):
                    return mkfactory(v,int64)
                elif isinstance(v[0],float):
                    return mkfactory(v,float64)
                elif isinstance(v[0],int):
                    return mkfactory(v,int32)
                else:
                    raise TypeError("Invalid type for array")
            else:   
                raise TypeError("Invalid type identifier %s" % str(t))

    return ndarray(mkfactory(v,t)(v))
    
class ndarray:
    def __init__(self,datap):
        if   isinstance(datap,ctypes.Array):
            self._datap = datap
            self.__length = len(datap)
        elif isinstance(datap,ndarray):
            self._datap = datap._datap
            self.__length = len(self._datap)
        else:
            raise TypeError("Invalid type for ndarray")
        self.dtype = self._datap._type_

    def all_eq(self,seq):
        if hasattr(seq,'__len__'):
            return all([ i==j for i,j in zip(self._datap,seq)])
        else:
            j = seq
            return all([ i==j for i in self._datap])
    def __len__(self):
        return int(self._datap.__len__())
    def _createnative(self,size): 
        return (self.dtype * size)()
    def getdatacptr(self): 
        return ctypes.cast(self._datap,ctypes.POINTER(self.dtype))
    def getsteplength(self): return 1
    def _sameclass(self,other): return False
    def _backcast(self,val): 
        return val
    def __getitem__ (self,key):
      if isinstance(key,tuple):
        if len(key) != 1:
          raise TypeError("Index must be one-dimensional")
        key, = key

      if isinstance(key,slice):
          try:
              val = operator.getitem(self._datap,key)
          except TypeError:              
              if key.step:
                  raise TypeError("Key cannot have a step size other than 1")
              val = operator.getslice(self._datap,key.start,key.stop)
          value = (self.dtype * len(val))()
          value[:] = val
          return array(value)
      else:
        return self._backcast(self._datap[key])
    def __setitem__ (self,key,value):
      if isinstance(key,slice):
        if hasattr(value,'__getitem__'):
            try:
                operator.setitem(self._datap,key,value) # works for 2.6+
            except TypeError:
                operator.setslice(self._datap,key.start,key.stop, value) # deprecated in 2.6, works in 2.5
                #self._datap.__setitem__(key,value)
        else:
            raise TypeError("Value must be a sequence")
      else:
        self._datap[key] = self.dtype(value)
    def __repr__(self):
      data = self._datap
      return '%s(%s)' % (self.__class__.__name__,','.join([repr(i) for i in data[:len(self)]]))
    def __str__(self):
      data = self._datap
      if len(self) > 50:
        return '%s(%s,...)' % (self.__class__.__name__,','.join([str(i) for i in data[:50]]))
      else:
        return '%s(%s)' % (self.__class__.__name__,','.join([str(i) for i in data[:len(self)] ]))


class doublearray(ndarray):
    def __init__(self,data_):
        if isinstance(data_,ctypes.Array):
            if data_._type_ is ctypes.c_double:
                d = data_
            else:
                d = (ctypes.c_double * len(data_))(*data_)
        else:
            d = (ctypes.c_double * len(data_))(*data_)

        ndarray.__init__(self,d)

class int32array(ndarray):
    def __init__(self,data_):
        if isinstance(data_,ctypes.Array):
            if data_._type_ is ctypes.c_int:
                d = data_
            else:
                d = (ctypes.c_int * len(data_))(*data_)
        else:
            d = (ctypes.c_int * len(data_))(*data_)

        ndarray.__init__(self,d)

class int64array(ndarray):
    def __init__(self,data_):
        if isinstance(data_,ctypes.Array):
            if data_._type_ is ctypes.c_longlong:
                d = data_
            else:
                d = (ctypes.c_longlong * len(data_))(*data_)
        else:
            d = (ctypes.c_longlong * len(data_))(*data_)

        ndarray.__init__(self,d)
        
def zeros(n,t=int32):    
    f = None
    if   t is int64   or t is long:  f = ctypes.c_longlong * n
    elif t is int32   or t is int:   f = ctypes.c_int * n
    elif t is float64 or t is float: f = ctypes.c_double * n
    else:
        raise TypeError("Invalid type identifier")
    r = array(f())
    return r

def ones(n,t=int32):
    if   t is int64   or t is long:  f,d = ctypes.c_longlong,1
    elif t is int32   or t is int:   f,d = ctypes.c_int * n,1
    elif t is float64 or t is float: f,d = ctypes.c_double * n,1.0
    else:
        raise TypeError("Invalid type identifier")

    return array(f(*([d]*n)))


def arange(a,b=None,step=None):
    if b is None:
        b = a
        a = 0
    if step is not None:
        return int32array(xrange(a,b,step))
    else:
        return int32array(xrange(a,b))
    
