import sys
import mosek.array
from mosek import array
#import numpy
import operator
import random
import new

class AbstractMethodError(Exception): pass

def is_basestring(s):
  return isinstance(s,basestring)


class PrintTrace:
  def __enter__(self):
    return self
  def __exit__(self,exc_type,exc_value,tb):
    if (exc_type and exc_value and tb):
      import traceback    
      traceback.print_tb(tb)
      print exc_value
def printtrace(f):
  def _(*args):
    with PrintTrace() as _:
      return f(*args)
  return _

if sys.maxint > 0x7fffffff:
  int64 = int
  int32 = int
else:
  int64 = long
  int32 = int


class Tools:

  @staticmethod
  def __arraycopy(src,srcoffset,tgt,tgtoffset,size):
    if len(tgt) < tgtoffset+size or len(src) < srcoffset+size:
      raise IndexError("Copy slice out of range")
    tgt[tgtoffset:tgtoffset+size] = src[srcoffset:srcoffset+size]


  _arraycopy__3II_3III =  __arraycopy
  _arraycopy__3LI_3LII = __arraycopy
  _arraycopy__3_3II_3_3III = __arraycopy
  _arraycopy__3_3FI_3_3FII = __arraycopy
  _arraycopy__3FI_3FII = __arraycopy
  _arraycopy__3IL_3ILL = __arraycopy
  _arraycopy__3LL_3LLL = __arraycopy
  _arraycopy__3FL_3FLL = __arraycopy

  @staticmethod
  #def __arrayclone(a): return array.array(a)
  def __arrayclone(a): return a[:]

  _arraycopy__3F = __arrayclone
  _arraycopy__3I = __arrayclone
  _arraycopy__3L = __arrayclone
  _arraycopy__3S = __arrayclone

  @staticmethod
  def __arraylength(a): return len(a)

  _arraylength__3Omosek_fusion_Variable_2   = __arraylength
  _arraylength__3Omosek_fusion_Constranit_2 = __arraylength  
  _arraylength__3Omosek_fusion_Matrix_2     = __arraylength  
  _arraylength__3S   = __arraylength  
  _arraylength__3I   = __arraylength  
  _arraylength__3L   = __arraylength  
  _arraylength__3F   = __arraylength  
  _arraylength__3_3F = __arraylength  
  _arraylength__3_3I = __arraylength  
  _uarraylength__3I  = __arraylength 
  _uarraylength__3L  = __arraylength 
  _uarraylength__3F  = __arraylength 
  _uarraylength__3Omosek_fusion_Variable_2   = __arraylength 
  _uarraylength__3Omosek_fusion_Constraint_2 = __arraylength 
  _uarraylength__3Omosek_fusion_Matrix_2     = __arraylength 
  _uarraylength__3_3S = __arraylength 
  _uarraylength__3_3F = __arraylength 
  _uarraylength__3_3I = __arraylength 

  @staticmethod
  def __matrixheight(m): return len(m)
  @staticmethod
  def __matrixwidth(m): return 0 if len(m) == 0 else len(m[0])
  
  _matrixheight__3_3F = __matrixheight
  _matrixwidth__3_3F  = __matrixwidth
  _matrixheight__3_3Omosek_fusion_Matrix_2 = __matrixheight
  _matrixwidth__3_3Omosek_fusion_Matrix_2  = __matrixwidth
 
  @staticmethod   
  def _range_L (last): return array.arange(last)
  @staticmethod   
  def _range_LL(first,last): return array.arange(first,last)
  @staticmethod   
  def _range_LLL(first,last,step): return array.arange(first,last,step)

  @staticmethod   
  def _range_I (last): return array.arange(last)
  @staticmethod   
  def _range_II(first,last): return array.arange(first,last)
  @staticmethod   
  def _range_III(first,last,step): return array.arange(first,last,step)

  @staticmethod   
  def _range_L (last): return range(last)
  @staticmethod   
  def _range_LL(first,last): return range(first,last)
  @staticmethod   
  def _range_LLL(first,last,step): return range(first,last,step)

  @staticmethod   
  def range(*args): return array.arange(*args)

  @staticmethod   
  def _zeros_I (num): return  array.zeros(num,float)
  @staticmethod   
  def _zeros_II (dimi,dimj): return  array.zeros((dimi,dimj),float)
  #def zeros (dimi,dimj=None):
  #  if dimj is None: return _zeros_I(dimi)
  #  else: _zeros_II(dimi,dimj)
  @staticmethod   
  def _ones_I(num):  return  array.ones(num,float)
  
  @staticmethod   
  def _asint_U(i):   return i
  @staticmethod   
  def _asuint_I(i):  return i
  @staticmethod   
  def _asint_J(i):   return i
  @staticmethod   
  def _asuint_L(i):  return i
  @staticmethod   
  def _as64bit_U(i): return long(i)
  @staticmethod   
  def _as64bit_I(i): return long(i)
  @staticmethod   
  def _as32bit_J(i): return int(i)
  @staticmethod   
  def _as32bit_L(i): return int(i)

  @staticmethod   
  def _sort__3III(val,first,last):
    tmp = list(val[first:last])
    tmp.sort()
    val[first:last] = tmp
  
  _sort__3LII = _sort__3III
    
  __rand = random.Random()
  @staticmethod   
  def _randInt_I(max):
    #return random.randint(0,max-1)
    return Tools.__rand.randint(0,max-1)
    
  @staticmethod   
  def _argsort__3L_3I_3LII(perm,val1,val2,first,last):
    p = list(perm[first:last])
    p.sort(lambda lhs,rhs: cmp(val1[lhs],val1[rhs]) or cmp(val2[lhs],val2[rhs]))
    perm[first:last] = p

  @staticmethod   
  def _argsort__3L_3I_3I(perm,val1,val2):
    Tools._argsort__3I_3I_3III(perm,val1,val2,0,len(perm))
  
  @staticmethod   
  def _argsort__3L_3I(perm,vals):
    Tools._argsort__3I_3III(perm,vals,0,len(perm))

  @staticmethod   
  def _argsort__3L_3III(perm,val,first,last):
    p = list(perm[first:last])
    p.sort(lambda lhs,rhs: cmp(val[lhs],val[rhs]))
    perm[first:last] = p

  @staticmethod
  def _sort__3I_II (vals,first,last):
    tmp = vals[first:last]
    tmp.sort()
    vals[first:last] = tmp

  @staticmethod   
  def _makevector_FI(val,num): return array.doublearray([val] * num)
  @staticmethod   
  def _makevector_II(val,num): return array.int32array([val] * num)
  @staticmethod   
  def _makevector_LI(val,num): return array.int64array([long(val)] * num)
  @staticmethod   
  def _repeatrange_III(first,last,num):
    res = array.zeros((last-first)*num,array.int32)
    ra = array.arange(first,last)
    l = last-first
    for i in range(num):
      res[i*l:(i+1)*l] = ra
    return res
  @staticmethod   
  def _repeatrange_LLL(first,last,num):
    res = array.zeros((last-first)*num,array.int64)
    ra = array.arange(first,last)
    l = last-first
    for i in range(num):
      res[i*l:(i+1)*l] = ra
    return res

#    public static method vectoradd   :: [double] (var lhs :: [double],var rhs :: [double]);
  @staticmethod 
  def _vectoradd__3F_3F (v1,v2):
    return array.array([ v1[i]+v2[i] for i in xrange(len(v1)) ])
  @staticmethod 
  def _vectorsub__3F_3F (v1,v2):
    return array.array([ v1[i]-v2[i] for i in xrange(len(v1)) ])
    #return v1-v2
#    public static method vectorsub   :: [double] (var lhs :: [double],var rhs :: [double]);
  @staticmethod 
  def _vectorneg__3F(v):
    return array.array([ -i for i in v ])
    #return -v
#    public static method dot         :: double (var lhs :: [double],var rhs :: [double]);
#    public static method sum         :: double (var rhs :: [double]);
  @staticmethod
  def  _stringvalue_I(v): return str(v)
  @staticmethod
  def  _stringvalue_L(v): return str(v)



  @staticmethod
  def _toDouble_S(v):    
    return float(v)
  
  @staticmethod
  def _toInt_S(v):
    return int(v)


#
#
#
#    public static method joinvals    :: string (var vals :: [int32]);
#

#class BaseException(Exception):
#  def _toString_ (self): return str(self)
#
#class BaseError(Exception):
#  def _toString_ (self): return str(self)
#
#class UnexpectedError(Exception):
#  def _toString_ (self): return str(self)

class StringBuffer:
  def __init__(self):
    self.__buf = []
  def __a(self,v):
    self.__buf.append(str(v))
    return self 
  def __a_array(self,v):
    if v is None: self.__buf.append("None")
    else: self.__buf.append(str(v))
    return self

  _a_I   = __a
  _a_L   = __a
  _a_U   = __a
  _a_F   = __a
  _a_S   = __a
  _a_B   = __a
  _a__3I = __a_array
  def _a__3L(self,v):
    if v is not None: self.__buf.extend([ '[',','.join(['%i' % i for i in v]),']'])
    else:             self.__buf.append('None')  
    return self
  def _a__3F(self,v):
    if v is not None: self.__buf.extend([ '[',','.join(['%g' % i for i in v]),']'])
    else:             self.__buf.append('None')  
    return self
  _a__3S = __a_array
  def _lf_(self): 
    self.__buf.append('\n')
    return self
  def _clear_ (self):
    self.__buf = []
    return self
  def _toString_(self):
    return ''.join(self.__buf)
  def _toString_(self):
    return ''.join(self.__buf)
  def _consolePrint_(self):
    print(self._toString_())
    self._clear_()
    return(self)
  
class IntMap:
  def __init__ (self):
    self.__d = {}
  def _hasItem_L(self,key): return self.__d.has_key(key)
  def _getItem_L(self,key): return self.__d[key]
  def _setItem_LI(self,key,val): self.__d[key] = val


class OutputFileStream:
  def __init__(self,filename):
    try:
      self.__outfile = open(filename,'wt')
    except IOError,e:
      raise mosek.fusion.IOError(str(e))
  def _write_S(self,v): 
    self.__outfile.write(str(v))
    return self
  def _write_I(self,v): 
    self.__outfile.write(str(v))
    return self
  def _write_L(self,v): 
    self.__outfile.write(str(v))
    return self
  def _write_F(self,v): 
    self.__outfile.write(str(v))
    return self
  def _lf_(self): 
    self.__outfile.write('\n')
    self.__outfile.flush()
    return self
  def _close_(self): self.__outfile.close()


class BinaryOutputStream:
  def __init__(self,filename):
    try:
      self.__outfile = open(filename,'bt')
    except IOError,e:
      raise mosek.fusion.IOError(str(e))
  def _write_S(self,v): 
    s = v.encode('utf-8')
    self.__outfile.write(struct.pack('<i',len(s)))
    self.__outfile.write(s)
    return self
  def _write_I(self,v): 
    self.__outfile.write(struct.pack('<i',v))
    return self
  def _write_L(self,v): 
    self.__outfile.write(struct.pack('<q',v))
    return self
  def _write_F(self,v): 
    self.__outfile.write(struct.pack('<d',v))
    return self
  def _write_B(self,v): 
    self.__outfile.write(struct.pack('<b',v))
    return self
  def _write__3I(self,v): 
    self.__outfile.write(struct.pack('<%di' % len(v),v))
    return self
  def _write__3L(self,v): 
    self.__outfile.write(struct.pack('<%dq' % len(v),v))
    return self
  def _write__3F(self,v): 
    self.__outfile.write(struct.pack('<%dd' % len(v),v))
    return self
  def _write__3B(self,v):
    self.__outfile.write(''.join([ chr(c) for c in v]))
    return self

  

  def _close_(self): self.__outfile.close()


def new_object_array (size): return [ None ] * size
def new_basestr_array (size): return [None] * size
def new_bool_array (size): return array.zeros(size,bool)
def new_int32_array (size): return array.zeros(size,array.int32)
def new_int64_array (size): return array.zeros(size,array.int64)
def new_double_array (size): return array.zeros(size,array.float64)
def isArray (v): 
  return v is None or operator.isSequenceType(v)
  #return v is None or isinstance(v,numpy.ndarray) or isinstance(v,mosek.array.ndarray) or isinstance(v,list)
def is_int(v):
  return isinstance(v,int) or isinstance(v,long)
def is_long(v):
  return isinstance(v,int) or isinstance(v,long)
def is_float(v):
  return isinstance(v,float)

# Argument checking methods
def methodargs(*convs):
  def _transform(func):
    def _conv(*args):
      try:
        if len(args) != len(convs):
          raise ValueError("Expected %d arguments, got %d" % (len(conv),len(args)))
        return func(*[ c(a) for (a,c) in zip(args,convs) ])
      except Exception, e:
        raise
        #raise ValueError('%s:%d: %s' % (func.func_code.co_filename,func.func_code.co_firstlineno,e))
    return _conv
  return _transform

def arg_id(a): return a
def arg_array(ndim,t):
  def _conv(a):
    if a is None: return None
    # test depth:
    v = a
    d = 0
    try:
      while not isinstance(v,basestring):
        v = v[0]
        d += 1
    except:
      pass
    if d != ndim:
      raise ValueError("Expected array of %d dimensions, got %d" % (ndim,d))
    if t is arg_id:
      return array.array(a)
    else:
      return array.array(a,t)
  return _conv

def abstractmethod(f):
  def _transform(*args):
    raise AbstractMethodError("Cannot invoke abstract method %s" % (f.__name__))

arg_bool   = bool
arg_int32  = array.int32
arg_int64  = array.int64
arg_double = array.float64
arg_string = unicode

class FusionException(Exception):
  def _toString_ (self):
    return '%s: %s' % (self.__class__.__name__,str(self))
  def toString(self):
    return '%s: %s' % (self.__class__.__name__,str(self))
class Debug:
  @staticmethod 
  def _o_(): return Debug()
  def p(self,v): 
    sys.stdout.write(str(v))
    return self
  _p_S = p
  _p_I = p
  _p_L = p
  _p_F = p
  _p_B = p

  def p_array (self,v):
    if v is None:
        sys.stdout.write('None')
    else:
        sys.stdout.write('[ %s ]' % ','.join([str(i) for i in v]))
    return self

  _p__3I = p_array
  _p__3L = p_array
  _p__3F = p_array

  def _lf_(self): 
    sys.stdout.write("\n")
    return self



def newobject(cls,init,*args):
    tmp = new.instance(cls)
    init(tmp,*args)
    return tmp

