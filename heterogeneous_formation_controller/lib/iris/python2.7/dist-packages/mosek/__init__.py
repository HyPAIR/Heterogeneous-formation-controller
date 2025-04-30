"""
    Mosek/Python Module. A Python interface to Mosek.
   
    Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
"""
import weakref
import sys
import threading
import os
if sys.version_info[0] != 2 or sys.version_info[1] < 5:
    raise ImportError('Invalid python version')
try:
    import ctypes
except ImportError:
    raise ImportError('PyMosek module requires ctypes')

import new
import re
import exceptions
from mosek import array

# Due to a bug in some python versions, lookup may fail in a multithreaded context.
import codecs
codecs.lookup('utf-8')

# Import numpy if available, otherwise create a numpy stub
try:
    import numpy
except ImportError:
    class numpy:
        @staticmethod
        def dtype(*args):
            return False
        class ndarray:
            """
            This is a dummy class. It is used as a stub when numpy is not present.
            """
            pass

def __makelibname(base):
    import platform as __pf
    libname = None
    sysname = __pf.system()
        
    if  sysname == 'Darwin':
        libname = 'lib%s.dylib' % base
    elif sysname == 'Windows':
        libname = '%s.dll' % base
    elif sysname == 'Linux':
        libname = 'lib%s.so' % base
    else: # assume linux/posix
        raise ImportError('Unknown system "%s"' % sysname)
    del __pf
    return libname

class TypeAcceptError(Exception):
    pass

class MosekException(Exception): pass

class SCoptException(MosekException): pass

class Exception(MosekException):
    def __init__(self,res,msg):
        MosekException.__init__(self,msg)
        self.msg   = msg
        self.errno = res
    def __str__(self):
        return "(%d) %s" % (self.errno,self.msg)
        
class Error(Exception):
    pass

class Warning(Exception):
    pass


def _make_intvector(v):
    """
    check validity of data and create an int32 array. if the argument is
    already a valid array, simply return that.
    """
    if v is None: return None
    if isinstance(v,numpy.ndarray):
        if v.ndim != 1:
            raise TypeAcceptError('expected a 1-dimensional array')
        elif v.dtype.type is numpy.int32:
            return v
        else:
            return array.array(v,array.int32)
    elif isinstance(v,array.ndarray):
        if   v.dtype is array.int32:
            return v
        else:
            return array.array(v,array.int32)
    else:
        return array.array(v,array.int32)

def _make_longvector(v):
    """
    check validity of data and create an int64 array. if the argument is
    already a valid array, simply return that.
    """
    if v is None: return None
    if isinstance(v,numpy.ndarray):
        if v.ndim != 1:
            raise TypeAcceptError('expected a 1-dimensional array')
        elif v.dtype.type is numpy.int64:
            return v
        else:
            return array.array(v,array.int64)
    elif isinstance(v,array.ndarray):
        if   v.dtype is array.int64:
            return v
        else:
            return array.array(v,array.int64)
    else:
        return array.array(v,array.int64)

def _make_doublevector(v):
    """
    Check validity of data and create an int32 array. If the argument is
    already a valid array, simply return that.
    """
    if v is None: return None
    if isinstance(v,numpy.ndarray):
        if v.ndim != 1:
            raise TypeAcceptError('Expected a 1-dimensional array')
        elif v.dtype.type is array.float64:
            return v
        else:
            return array.array(v,array.float64)
    elif isinstance(v,array.ndarray):
        if   v.dtype is array.float64:
            return v
        else:
            return array.array(v,array.float64)
    else:
        return array.array(v,array.float64)

def _check_stringlist(v):
    for i in v:
        if not (isinstance(i,str) or isinstance(i,unicode)):
            raise TypeAcceptError('expected an array of string objests')
    return v 
def _check_taskvector(v):
    for i in v:
        if not (i is None or isinstance(i,Task)):
            raise TypeAcceptError('expected an array of mosek.task objests')
    return v 
    
def _accept_intvector(v):
    return v

def _accept_str(v):
    if not (isinstance(v,str) or isinstance(v,unicode)):
        raise TypeAcceptError("Expected a string argument")
    return v
def _make_anyenumvector(e):
    def acceptvector(v):
        if v is None: return None
        for i in v:
            if not isinstance(i,e):
                raise TypeAcceptError("Expected an array of %s values" % e.__name__)
        return array.array(v,array.int32)
    return acceptvector


def _accept_longvector(v):
    return v

def _accept_doublevector(v):
    return v

def _accept_any(v):
    return v



_make_int = int
_make_long = long
_make_double = float
def _make_str(v):
    if isinstance(v,str) or isinstance(v,unicode):
        return v
    else:
        return unicode(v)


def _accept_anyenum(e):
    def acceptenum(v):
        if isinstance(v,e):
            return v
        else:
            raise TypeError('Expected an %s enum type' % e.__name__)
    return acceptenum

def accepts(*argtlst):
    """
    Decorator for checking function arguments.
    """
    def acceptsfun(fun):
        def accept(*args):
            if len(args) != len(argtlst):
                raise TypeError('Expected %d argument(s) (%d given)' % (len(argtlst), len(args)))
            try:
                return fun(*[ t(a) for (t,a) in zip(argtlst,args) ])
            except TypeAcceptError, e:
                raise TypeError(e)
        accept.__doc__ = fun.__doc__
        accept.__name__ = fun.__name__
        return accept

    return acceptsfun

    


        
    

class EnumBase(int):
    """
    Base class for enums.
    """
    enumnamere = re.compile(r'[a-zA-Z][a-zA-Z0-9_]*$')
    def __new__(cls,value):
        if isinstance(value,int):
            return cls._valdict[value]
        elif isinstance(value,str) or isinstance(value,unicode):
            return cls._namedict[value.split('.')[-1]]
        else:
            raise TypeError("Invalid type for enum construction")
    def __str__(self):
        return '%s.%s' % (self.__class__.__name__,self.__name__)
    def __repr__(self):
        return self.__name__

    @classmethod
    def __getitem__(cls,k):
        return cls._values[k]

    @classmethod
    def __len__(cls):
        return len(cls._values)

    @classmethod
    def members(cls):
        return iter(cls._values)

    @classmethod
    def _initialize(cls, names,values=None):
        for n in names:
            if not cls.enumnamere.match(n):
                raise ValueError("Invalid enum item name '%s' in %s" % (n,cls.__name__))
        if values is None:
            values = range(len(names))
        if len(values) != len(names):
            raise ValueError("Lengths of names and values do not match")
        
        items = []
        for (n,v) in zip(names,values):
            item = int.__new__(cls,v)
            item.__name__ = n
            setattr(cls,n,item)
            items.append(item)

        cls._values   = items
        cls.values    = items
        cls._namedict = dict([ (v.__name__,v) for v in items ])
        cls._valdict  = dict([ (v,v) for v in items ]) # map int -> enum value (sneaky, eh?)
        

def Enum(name,names,values=None):
    """
    Create a new enum class with the given names and values.
    
    Parameters:
     [name]   A string denoting the name of the enum.
     [names]  A list of strings denoting the names of the individual enum values.
     [values] (optional) A list of integer values of the enums. If given, the
       list must have same length as the [names] parameter. If not given, the
       default values 0, 1, ... will be used.
    """
    e = new.classobj(name,(EnumBase,),{})
    e._initialize(names,values)
    return e
    

# module initialization
import platform
__libloadpath = None
try:
    __libname = __makelibname('mosekxx7_1')
    __libsupport = []
    if platform.system() == 'Windows':
        __library_factory__ = ctypes.WinDLL
        __callback_factory__ = ctypes.WINFUNCTYPE # stdcall

        if ctypes.sizeof(ctypes.c_void_p) == 4:
          __libsupport.append('mosek7_1.dll')
        else:
          __libsupport.append('mosek64_7_1.dll')
    elif platform.system() in [ 'Darwin', 'Linux' ]:
        __library_factory__ = ctypes.CDLL 
        __callback_factory__ = ctypes.CFUNCTYPE # cdecl

    try:
        # try the module directory (when installed)
        for l in __libsupport: __library_factory__(os.path.join(os.path.dirname(__file__),l))
        __library__ = __library_factory__(os.path.join(os.path.dirname(__file__),__libname))
        __libloadpath = os.path.join(os.path.dirname(__file__))
    except OSError:
      try:
        # try relative path of the binary directory (when not installed)
        for l in __libsupport: __library_factory__(os.path.join(os.path.dirname(__file__),'..','..','..','bin',l))
        __library__ = __library_factory__(os.path.join(os.path.dirname(__file__),'..','..','..','bin',__libname))
        __libloadpath = os.path.normpath(os.path.join(os.path.dirname(__file__),'..','..','..','bin'))
      except OSError:
          # try default search path
          __library__ = __library_factory__(__libname)
except OSError:
    raise ImportError('Failed to import dll "%s"' % __libname)

__progress_cb_type__ = __callback_factory__(ctypes.c_int,  # return int
                                            ctypes.POINTER(ctypes.c_void_p), # task
                                            ctypes.c_void_p,  # handle
                                            ctypes.c_int,  # caller
                                            ctypes.POINTER(ctypes.c_double),  # dinf
                                            ctypes.POINTER(ctypes.c_int), # iinf
                                            ctypes.POINTER(ctypes.c_longlong))# liinf
__stream_cb_type__   = __callback_factory__(None, ctypes.c_void_p, ctypes.c_char_p)

__library__.MSK_XX_makeenv.restype      =   ctypes.c_int
__library__.MSK_XX_makeenv.argtypes     = [ ctypes.POINTER(ctypes.c_void_p), 
                                            ctypes.c_char_p ] 
__library__.MSK_XX_deleteenv.argtypes   = [ ctypes.POINTER(ctypes.c_void_p) ] # envp
__library__.MSK_XX_deleteenv.restype    =   ctypes.c_int
__library__.MSK_XX_maketask.argtypes    = [ ctypes.c_void_p,# env
                                         ctypes.c_int, # maxnumcon
                                         ctypes.c_int, # maxnumvar
                                         ctypes.POINTER(ctypes.c_void_p)] # taskp
__library__.MSK_XX_maketask.restype     =   ctypes.c_int
__library__.MSK_XX_deletetask.argtypes  = [ ctypes.POINTER(ctypes.c_void_p) ] # envp
__library__.MSK_XX_deletetask.restype   =   ctypes.c_int
__library__.MSK_XX_putcallbackfunc.argtypes      = [ ctypes.c_void_p, __progress_cb_type__, ctypes.c_void_p ]
__library__.MSK_XX_putcallbackfunc.restype       =   ctypes.c_int
__library__.MSK_XX_linkfunctotaskstream.argtypes = [ ctypes.c_void_p,    # task 
                                                  ctypes.c_int,       # whichstream
                                                  ctypes.c_void_p,    # handle
                                                  __stream_cb_type__ ] # func
__library__.MSK_XX_linkfunctotaskstream.restype  =   ctypes.c_int
__library__.MSK_XX_linkfunctoenvstream.argtypes  = [ ctypes.c_void_p,    # env 
                                                  ctypes.c_int,       # whichstream
                                                  ctypes.c_void_p,    # handle
                                                  __stream_cb_type__ ] # func
__library__.MSK_XX_linkfunctoenvstream.restype   =   ctypes.c_int
__library__.MSK_XX_clonetask.restype     = ctypes.c_int
__library__.MSK_XX_clonetask.argtypes    = [ ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p) ]
__library__.MSK_XX_getlasterror64.restype  = ctypes.c_int
__library__.MSK_XX_getlasterror64.argtypes = [ ctypes.c_void_p, # task
                                               ctypes.POINTER(ctypes.c_int), # lastrescode
                                               ctypes.c_int64, # maxlen
                                               ctypes.POINTER(ctypes.c_size_t), # msg len
                                               ctypes.c_char_p, ] # msg
__library__.MSK_XX_putresponsefunc.restype  = ctypes.c_int
__library__.MSK_XX_putresponsefunc.argtypes = [ ctypes.c_void_p,  # task
                                             ctypes.c_void_p,  # func
                                             ctypes.c_void_p ] # handle
__library__.MSK_XX_enablegarcolenv.argtypes = [ ctypes.c_void_p ] 
__library__.MSK_XX_enablegarcolenv.restype  =   ctypes.c_int 

__library__.MSK_XX_putnlfunc.restype = ctypes.c_int
__library__.MSK_XX_putnlfunc.argtypes = [ ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p ]

__library__.MSK_XX_analyzeproblem.restype  = ctypes.c_int
__library__.MSK_XX_analyzeproblem.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_analyzenames.restype  = ctypes.c_int
__library__.MSK_XX_analyzenames.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_analyzesolution.restype  = ctypes.c_int
__library__.MSK_XX_analyzesolution.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_initbasissolve.restype  = ctypes.c_int
__library__.MSK_XX_initbasissolve.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_solvewithbasis.restype  = ctypes.c_int
__library__.MSK_XX_solvewithbasis.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_basiscond.restype  = ctypes.c_int
__library__.MSK_XX_basiscond.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_appendcons.restype  = ctypes.c_int
__library__.MSK_XX_appendcons.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_appendvars.restype  = ctypes.c_int
__library__.MSK_XX_appendvars.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_removecons.restype  = ctypes.c_int
__library__.MSK_XX_removecons.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_removevars.restype  = ctypes.c_int
__library__.MSK_XX_removevars.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_removebarvars.restype  = ctypes.c_int
__library__.MSK_XX_removebarvars.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_removecones.restype  = ctypes.c_int
__library__.MSK_XX_removecones.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_appendbarvars.restype  = ctypes.c_int
__library__.MSK_XX_appendbarvars.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_appendcone.restype  = ctypes.c_int
__library__.MSK_XX_appendcone.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_double,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_appendconeseq.restype  = ctypes.c_int
__library__.MSK_XX_appendconeseq.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_double,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_appendconesseq.restype  = ctypes.c_int
__library__.MSK_XX_appendconesseq.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int32),ctypes.c_int32]
__library__.MSK_XX_chgbound.restype  = ctypes.c_int
__library__.MSK_XX_chgbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_getaij.restype  = ctypes.c_int
__library__.MSK_XX_getaij.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getapiecenumnz.restype  = ctypes.c_int
__library__.MSK_XX_getapiecenumnz.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getacolnumnz.restype  = ctypes.c_int
__library__.MSK_XX_getacolnumnz.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getacol.restype  = ctypes.c_int
__library__.MSK_XX_getacol.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getarownumnz.restype  = ctypes.c_int
__library__.MSK_XX_getarownumnz.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getarow.restype  = ctypes.c_int
__library__.MSK_XX_getarow.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getaslicenumnz64.restype  = ctypes.c_int
__library__.MSK_XX_getaslicenumnz64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getaslice64.restype  = ctypes.c_int
__library__.MSK_XX_getaslice64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getarowslicetrip.restype  = ctypes.c_int
__library__.MSK_XX_getarowslicetrip.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getacolslicetrip.restype  = ctypes.c_int
__library__.MSK_XX_getacolslicetrip.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getconbound.restype  = ctypes.c_int
__library__.MSK_XX_getconbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getvarbound.restype  = ctypes.c_int
__library__.MSK_XX_getvarbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getbound.restype  = ctypes.c_int
__library__.MSK_XX_getbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getconboundslice.restype  = ctypes.c_int
__library__.MSK_XX_getconboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getvarboundslice.restype  = ctypes.c_int
__library__.MSK_XX_getvarboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getboundslice.restype  = ctypes.c_int
__library__.MSK_XX_getboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putboundslice.restype  = ctypes.c_int
__library__.MSK_XX_putboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getcj.restype  = ctypes.c_int
__library__.MSK_XX_getcj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getc.restype  = ctypes.c_int
__library__.MSK_XX_getc.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getcfix.restype  = ctypes.c_int
__library__.MSK_XX_getcfix.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getcone.restype  = ctypes.c_int
__library__.MSK_XX_getcone.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getconeinfo.restype  = ctypes.c_int
__library__.MSK_XX_getconeinfo.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getcslice.restype  = ctypes.c_int
__library__.MSK_XX_getcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdouinf.restype  = ctypes.c_int
__library__.MSK_XX_getdouinf.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdouparam.restype  = ctypes.c_int
__library__.MSK_XX_getdouparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdualobj.restype  = ctypes.c_int
__library__.MSK_XX_getdualobj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getintinf.restype  = ctypes.c_int
__library__.MSK_XX_getintinf.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getlintinf.restype  = ctypes.c_int
__library__.MSK_XX_getlintinf.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getintparam.restype  = ctypes.c_int
__library__.MSK_XX_getintparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getmaxnumanz64.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumanz64.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getmaxnumcon.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumcon.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getmaxnumvar.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumvar.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getbarvarnamelen.restype  = ctypes.c_int
__library__.MSK_XX_getbarvarnamelen.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getbarvarname.restype  = ctypes.c_int
__library__.MSK_XX_getbarvarname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getbarvarnameindex.restype  = ctypes.c_int
__library__.MSK_XX_getbarvarnameindex.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_putconname.restype  = ctypes.c_int
__library__.MSK_XX_putconname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_putvarname.restype  = ctypes.c_int
__library__.MSK_XX_putvarname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_putconename.restype  = ctypes.c_int
__library__.MSK_XX_putconename.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_putbarvarname.restype  = ctypes.c_int
__library__.MSK_XX_putbarvarname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getvarnamelen.restype  = ctypes.c_int
__library__.MSK_XX_getvarnamelen.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getvarname.restype  = ctypes.c_int
__library__.MSK_XX_getvarname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getconnamelen.restype  = ctypes.c_int
__library__.MSK_XX_getconnamelen.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getconname.restype  = ctypes.c_int
__library__.MSK_XX_getconname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getconnameindex.restype  = ctypes.c_int
__library__.MSK_XX_getconnameindex.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getvarnameindex.restype  = ctypes.c_int
__library__.MSK_XX_getvarnameindex.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getconenamelen.restype  = ctypes.c_int
__library__.MSK_XX_getconenamelen.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getconename.restype  = ctypes.c_int
__library__.MSK_XX_getconename.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getconenameindex.restype  = ctypes.c_int
__library__.MSK_XX_getconenameindex.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumanz.restype  = ctypes.c_int
__library__.MSK_XX_getnumanz.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumanz64.restype  = ctypes.c_int
__library__.MSK_XX_getnumanz64.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getnumcon.restype  = ctypes.c_int
__library__.MSK_XX_getnumcon.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumcone.restype  = ctypes.c_int
__library__.MSK_XX_getnumcone.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumconemem.restype  = ctypes.c_int
__library__.MSK_XX_getnumconemem.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumintvar.restype  = ctypes.c_int
__library__.MSK_XX_getnumintvar.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumparam.restype  = ctypes.c_int
__library__.MSK_XX_getnumparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumqconknz.restype  = ctypes.c_int
__library__.MSK_XX_getnumqconknz.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumqconknz64.restype  = ctypes.c_int
__library__.MSK_XX_getnumqconknz64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getnumqobjnz64.restype  = ctypes.c_int
__library__.MSK_XX_getnumqobjnz64.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getnumvar.restype  = ctypes.c_int
__library__.MSK_XX_getnumvar.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumbarvar.restype  = ctypes.c_int
__library__.MSK_XX_getnumbarvar.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getmaxnumbarvar.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumbarvar.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getdimbarvarj.restype  = ctypes.c_int
__library__.MSK_XX_getdimbarvarj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getlenbarvarj.restype  = ctypes.c_int
__library__.MSK_XX_getlenbarvarj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getobjname.restype  = ctypes.c_int
__library__.MSK_XX_getobjname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getobjnamelen.restype  = ctypes.c_int
__library__.MSK_XX_getobjnamelen.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getprimalobj.restype  = ctypes.c_int
__library__.MSK_XX_getprimalobj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getprobtype.restype  = ctypes.c_int
__library__.MSK_XX_getprobtype.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getqconk64.restype  = ctypes.c_int
__library__.MSK_XX_getqconk64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getqobj.restype  = ctypes.c_int
__library__.MSK_XX_getqobj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getqobj64.restype  = ctypes.c_int
__library__.MSK_XX_getqobj64.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getqobjij.restype  = ctypes.c_int
__library__.MSK_XX_getqobjij.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolution.restype  = ctypes.c_int
__library__.MSK_XX_getsolution.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpbi.restype  = ctypes.c_int
__library__.MSK_XX_getpbi.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.c_int32]
__library__.MSK_XX_getdbi.restype  = ctypes.c_int
__library__.MSK_XX_getdbi.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdeqi.restype  = ctypes.c_int
__library__.MSK_XX_getdeqi.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.c_int32]
__library__.MSK_XX_getpeqi.restype  = ctypes.c_int
__library__.MSK_XX_getpeqi.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.c_int32]
__library__.MSK_XX_getinti.restype  = ctypes.c_int
__library__.MSK_XX_getinti.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpcni.restype  = ctypes.c_int
__library__.MSK_XX_getpcni.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdcni.restype  = ctypes.c_int
__library__.MSK_XX_getdcni.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolutioni.restype  = ctypes.c_int
__library__.MSK_XX_getsolutioni.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolsta.restype  = ctypes.c_int
__library__.MSK_XX_getsolsta.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getprosta.restype  = ctypes.c_int
__library__.MSK_XX_getprosta.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getskc.restype  = ctypes.c_int
__library__.MSK_XX_getskc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_getskx.restype  = ctypes.c_int
__library__.MSK_XX_getskx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_getxc.restype  = ctypes.c_int
__library__.MSK_XX_getxc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getxx.restype  = ctypes.c_int
__library__.MSK_XX_getxx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_gety.restype  = ctypes.c_int
__library__.MSK_XX_gety.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getslc.restype  = ctypes.c_int
__library__.MSK_XX_getslc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsuc.restype  = ctypes.c_int
__library__.MSK_XX_getsuc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getslx.restype  = ctypes.c_int
__library__.MSK_XX_getslx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsux.restype  = ctypes.c_int
__library__.MSK_XX_getsux.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsnx.restype  = ctypes.c_int
__library__.MSK_XX_getsnx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getskcslice.restype  = ctypes.c_int
__library__.MSK_XX_getskcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_getskxslice.restype  = ctypes.c_int
__library__.MSK_XX_getskxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_getxcslice.restype  = ctypes.c_int
__library__.MSK_XX_getxcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getxxslice.restype  = ctypes.c_int
__library__.MSK_XX_getxxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getyslice.restype  = ctypes.c_int
__library__.MSK_XX_getyslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getslcslice.restype  = ctypes.c_int
__library__.MSK_XX_getslcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsucslice.restype  = ctypes.c_int
__library__.MSK_XX_getsucslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getslxslice.restype  = ctypes.c_int
__library__.MSK_XX_getslxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsuxslice.restype  = ctypes.c_int
__library__.MSK_XX_getsuxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsnxslice.restype  = ctypes.c_int
__library__.MSK_XX_getsnxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getbarxj.restype  = ctypes.c_int
__library__.MSK_XX_getbarxj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getbarsj.restype  = ctypes.c_int
__library__.MSK_XX_getbarsj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putskc.restype  = ctypes.c_int
__library__.MSK_XX_putskc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_putskx.restype  = ctypes.c_int
__library__.MSK_XX_putskx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_putxc.restype  = ctypes.c_int
__library__.MSK_XX_putxc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putxx.restype  = ctypes.c_int
__library__.MSK_XX_putxx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_puty.restype  = ctypes.c_int
__library__.MSK_XX_puty.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putslc.restype  = ctypes.c_int
__library__.MSK_XX_putslc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsuc.restype  = ctypes.c_int
__library__.MSK_XX_putsuc.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putslx.restype  = ctypes.c_int
__library__.MSK_XX_putslx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsux.restype  = ctypes.c_int
__library__.MSK_XX_putsux.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsnx.restype  = ctypes.c_int
__library__.MSK_XX_putsnx.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putskcslice.restype  = ctypes.c_int
__library__.MSK_XX_putskcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_putskxslice.restype  = ctypes.c_int
__library__.MSK_XX_putskxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_putxcslice.restype  = ctypes.c_int
__library__.MSK_XX_putxcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putxxslice.restype  = ctypes.c_int
__library__.MSK_XX_putxxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putyslice.restype  = ctypes.c_int
__library__.MSK_XX_putyslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putslcslice.restype  = ctypes.c_int
__library__.MSK_XX_putslcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsucslice.restype  = ctypes.c_int
__library__.MSK_XX_putsucslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putslxslice.restype  = ctypes.c_int
__library__.MSK_XX_putslxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsuxslice.restype  = ctypes.c_int
__library__.MSK_XX_putsuxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsnxslice.restype  = ctypes.c_int
__library__.MSK_XX_putsnxslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbarxj.restype  = ctypes.c_int
__library__.MSK_XX_putbarxj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbarsj.restype  = ctypes.c_int
__library__.MSK_XX_putbarsj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolutioninf.restype  = ctypes.c_int
__library__.MSK_XX_getsolutioninf.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpviolcon.restype  = ctypes.c_int
__library__.MSK_XX_getpviolcon.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpviolvar.restype  = ctypes.c_int
__library__.MSK_XX_getpviolvar.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpviolbarvar.restype  = ctypes.c_int
__library__.MSK_XX_getpviolbarvar.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getpviolcones.restype  = ctypes.c_int
__library__.MSK_XX_getpviolcones.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdviolcon.restype  = ctypes.c_int
__library__.MSK_XX_getdviolcon.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdviolvar.restype  = ctypes.c_int
__library__.MSK_XX_getdviolvar.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdviolbarvar.restype  = ctypes.c_int
__library__.MSK_XX_getdviolbarvar.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getdviolcones.restype  = ctypes.c_int
__library__.MSK_XX_getdviolcones.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolutioninfo.restype  = ctypes.c_int
__library__.MSK_XX_getsolutioninfo.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getsolutionslice.restype  = ctypes.c_int
__library__.MSK_XX_getsolutionslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getreducedcosts.restype  = ctypes.c_int
__library__.MSK_XX_getreducedcosts.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getstrparam.restype  = ctypes.c_int
__library__.MSK_XX_getstrparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.c_char_p]
__library__.MSK_XX_getstrparamlen.restype  = ctypes.c_int
__library__.MSK_XX_getstrparamlen.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_gettasknamelen.restype  = ctypes.c_int
__library__.MSK_XX_gettasknamelen.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_gettaskname.restype  = ctypes.c_int
__library__.MSK_XX_gettaskname.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_getvartype.restype  = ctypes.c_int
__library__.MSK_XX_getvartype.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getvartypelist.restype  = ctypes.c_int
__library__.MSK_XX_getvartypelist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_inputdata64.restype  = ctypes.c_int
__library__.MSK_XX_inputdata64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_isdouparname.restype  = ctypes.c_int
__library__.MSK_XX_isdouparname.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_isintparname.restype  = ctypes.c_int
__library__.MSK_XX_isintparname.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_isstrparname.restype  = ctypes.c_int
__library__.MSK_XX_isstrparname.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_linkfiletotaskstream.restype  = ctypes.c_int
__library__.MSK_XX_linkfiletotaskstream.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p,ctypes.c_int32]
__library__.MSK_XX_relaxprimal.restype  = ctypes.c_int
__library__.MSK_XX_relaxprimal.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_primalrepair.restype  = ctypes.c_int
__library__.MSK_XX_primalrepair.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_toconic.restype  = ctypes.c_int
__library__.MSK_XX_toconic.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_optimizeconcurrent.restype  = ctypes.c_int
__library__.MSK_XX_optimizeconcurrent.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_void_p),ctypes.c_int32]
__library__.MSK_XX_optimizetrm.restype  = ctypes.c_int
__library__.MSK_XX_optimizetrm.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_printdata.restype  = ctypes.c_int
__library__.MSK_XX_printdata.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_printparam.restype  = ctypes.c_int
__library__.MSK_XX_printparam.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_commitchanges.restype  = ctypes.c_int
__library__.MSK_XX_commitchanges.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_putaij.restype  = ctypes.c_int
__library__.MSK_XX_putaij.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_putaijlist64.restype  = ctypes.c_int
__library__.MSK_XX_putaijlist64.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putacol.restype  = ctypes.c_int
__library__.MSK_XX_putacol.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putarow.restype  = ctypes.c_int
__library__.MSK_XX_putarow.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putarowlist64.restype  = ctypes.c_int
__library__.MSK_XX_putarowlist64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putacolslice64.restype  = ctypes.c_int
__library__.MSK_XX_putacolslice64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putacollist64.restype  = ctypes.c_int
__library__.MSK_XX_putacollist64.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbaraij.restype  = ctypes.c_int
__library__.MSK_XX_putbaraij.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getnumbarcnz.restype  = ctypes.c_int
__library__.MSK_XX_getnumbarcnz.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getnumbaranz.restype  = ctypes.c_int
__library__.MSK_XX_getnumbaranz.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarcsparsity.restype  = ctypes.c_int
__library__.MSK_XX_getbarcsparsity.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarasparsity.restype  = ctypes.c_int
__library__.MSK_XX_getbarasparsity.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarcidxinfo.restype  = ctypes.c_int
__library__.MSK_XX_getbarcidxinfo.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarcidxj.restype  = ctypes.c_int
__library__.MSK_XX_getbarcidxj.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getbarcidx.restype  = ctypes.c_int
__library__.MSK_XX_getbarcidx.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getbaraidxinfo.restype  = ctypes.c_int
__library__.MSK_XX_getbaraidxinfo.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbaraidxij.restype  = ctypes.c_int
__library__.MSK_XX_getbaraidxij.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getbaraidx.restype  = ctypes.c_int
__library__.MSK_XX_getbaraidx.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbarcblocktriplet.restype  = ctypes.c_int
__library__.MSK_XX_putbarcblocktriplet.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbarablocktriplet.restype  = ctypes.c_int
__library__.MSK_XX_putbarablocktriplet.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getnumbarcblocktriplets.restype  = ctypes.c_int
__library__.MSK_XX_getnumbarcblocktriplets.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarcblocktriplet.restype  = ctypes.c_int
__library__.MSK_XX_getbarcblocktriplet.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_getnumbarablocktriplets.restype  = ctypes.c_int
__library__.MSK_XX_getnumbarablocktriplets.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getbarablocktriplet.restype  = ctypes.c_int
__library__.MSK_XX_getbarablocktriplet.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbound.restype  = ctypes.c_int
__library__.MSK_XX_putbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_double]
__library__.MSK_XX_putboundlist.restype  = ctypes.c_int
__library__.MSK_XX_putboundlist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putconbound.restype  = ctypes.c_int
__library__.MSK_XX_putconbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_double]
__library__.MSK_XX_putconboundlist.restype  = ctypes.c_int
__library__.MSK_XX_putconboundlist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putconboundslice.restype  = ctypes.c_int
__library__.MSK_XX_putconboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putvarbound.restype  = ctypes.c_int
__library__.MSK_XX_putvarbound.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_double]
__library__.MSK_XX_putvarboundlist.restype  = ctypes.c_int
__library__.MSK_XX_putvarboundlist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putvarboundslice.restype  = ctypes.c_int
__library__.MSK_XX_putvarboundslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putcfix.restype  = ctypes.c_int
__library__.MSK_XX_putcfix.argtypes = [ctypes.c_void_p,ctypes.c_double]
__library__.MSK_XX_putcj.restype  = ctypes.c_int
__library__.MSK_XX_putcj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_putobjsense.restype  = ctypes.c_int
__library__.MSK_XX_putobjsense.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_getobjsense.restype  = ctypes.c_int
__library__.MSK_XX_getobjsense.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_putclist.restype  = ctypes.c_int
__library__.MSK_XX_putclist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putcslice.restype  = ctypes.c_int
__library__.MSK_XX_putcslice.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putbarcj.restype  = ctypes.c_int
__library__.MSK_XX_putbarcj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putcone.restype  = ctypes.c_int
__library__.MSK_XX_putcone.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_appendsparsesymmat.restype  = ctypes.c_int
__library__.MSK_XX_appendsparsesymmat.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getsymmatinfo.restype  = ctypes.c_int
__library__.MSK_XX_getsymmatinfo.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getnumsymmat.restype  = ctypes.c_int
__library__.MSK_XX_getnumsymmat.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_getsparsesymmat.restype  = ctypes.c_int
__library__.MSK_XX_getsparsesymmat.argtypes = [ctypes.c_void_p,ctypes.c_int64,ctypes.c_int64,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putdouparam.restype  = ctypes.c_int
__library__.MSK_XX_putdouparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_putintparam.restype  = ctypes.c_int
__library__.MSK_XX_putintparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_putmaxnumcon.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumcon.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_putmaxnumcone.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumcone.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_getmaxnumcone.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumcone.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_putmaxnumvar.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumvar.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_putmaxnumbarvar.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumbarvar.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_putmaxnumanz.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumanz.argtypes = [ctypes.c_void_p,ctypes.c_int64]
__library__.MSK_XX_putmaxnumqnz.restype  = ctypes.c_int
__library__.MSK_XX_putmaxnumqnz.argtypes = [ctypes.c_void_p,ctypes.c_int64]
__library__.MSK_XX_getmaxnumqnz64.restype  = ctypes.c_int
__library__.MSK_XX_getmaxnumqnz64.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_putnadouparam.restype  = ctypes.c_int
__library__.MSK_XX_putnadouparam.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_double]
__library__.MSK_XX_putnaintparam.restype  = ctypes.c_int
__library__.MSK_XX_putnaintparam.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int32]
__library__.MSK_XX_putnastrparam.restype  = ctypes.c_int
__library__.MSK_XX_putnastrparam.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_char_p]
__library__.MSK_XX_putobjname.restype  = ctypes.c_int
__library__.MSK_XX_putobjname.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_putparam.restype  = ctypes.c_int
__library__.MSK_XX_putparam.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_char_p]
__library__.MSK_XX_putqcon.restype  = ctypes.c_int
__library__.MSK_XX_putqcon.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putqconk.restype  = ctypes.c_int
__library__.MSK_XX_putqconk.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putqobj.restype  = ctypes.c_int
__library__.MSK_XX_putqobj.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putqobjij.restype  = ctypes.c_int
__library__.MSK_XX_putqobjij.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_putsolution.restype  = ctypes.c_int
__library__.MSK_XX_putsolution.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_putsolutioni.restype  = ctypes.c_int
__library__.MSK_XX_putsolutioni.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double]
__library__.MSK_XX_putsolutionyi.restype  = ctypes.c_int
__library__.MSK_XX_putsolutionyi.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_double]
__library__.MSK_XX_putstrparam.restype  = ctypes.c_int
__library__.MSK_XX_putstrparam.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_puttaskname.restype  = ctypes.c_int
__library__.MSK_XX_puttaskname.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_putvartype.restype  = ctypes.c_int
__library__.MSK_XX_putvartype.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_putvartypelist.restype  = ctypes.c_int
__library__.MSK_XX_putvartypelist.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int)]
__library__.MSK_XX_putvarbranchorder.restype  = ctypes.c_int
__library__.MSK_XX_putvarbranchorder.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_getvarbranchpri.restype  = ctypes.c_int
__library__.MSK_XX_getvarbranchpri.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_getvarbranchdir.restype  = ctypes.c_int
__library__.MSK_XX_getvarbranchdir.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_readdataformat.restype  = ctypes.c_int
__library__.MSK_XX_readdataformat.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_readdataautoformat.restype  = ctypes.c_int
__library__.MSK_XX_readdataautoformat.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_readparamfile.restype  = ctypes.c_int
__library__.MSK_XX_readparamfile.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_readsolution.restype  = ctypes.c_int
__library__.MSK_XX_readsolution.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_readsummary.restype  = ctypes.c_int
__library__.MSK_XX_readsummary.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_resizetask.restype  = ctypes.c_int
__library__.MSK_XX_resizetask.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int64,ctypes.c_int64]
__library__.MSK_XX_checkmemtask.restype  = ctypes.c_int
__library__.MSK_XX_checkmemtask.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int32]
__library__.MSK_XX_getmemusagetask.restype  = ctypes.c_int
__library__.MSK_XX_getmemusagetask.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int64),ctypes.POINTER(ctypes.c_int64)]
__library__.MSK_XX_setdefaults.restype  = ctypes.c_int
__library__.MSK_XX_setdefaults.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_solutiondef.restype  = ctypes.c_int
__library__.MSK_XX_solutiondef.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_deletesolution.restype  = ctypes.c_int
__library__.MSK_XX_deletesolution.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_startstat.restype  = ctypes.c_int
__library__.MSK_XX_startstat.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_stopstat.restype  = ctypes.c_int
__library__.MSK_XX_stopstat.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_appendstat.restype  = ctypes.c_int
__library__.MSK_XX_appendstat.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_onesolutionsummary.restype  = ctypes.c_int
__library__.MSK_XX_onesolutionsummary.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32]
__library__.MSK_XX_solutionsummary.restype  = ctypes.c_int
__library__.MSK_XX_solutionsummary.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_updatesolutioninfo.restype  = ctypes.c_int
__library__.MSK_XX_updatesolutioninfo.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_optimizersummary.restype  = ctypes.c_int
__library__.MSK_XX_optimizersummary.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_strtoconetype.restype  = ctypes.c_int
__library__.MSK_XX_strtoconetype.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_strtosk.restype  = ctypes.c_int
__library__.MSK_XX_strtosk.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_writedata.restype  = ctypes.c_int
__library__.MSK_XX_writedata.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_writetask.restype  = ctypes.c_int
__library__.MSK_XX_writetask.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_readtask.restype  = ctypes.c_int
__library__.MSK_XX_readtask.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_readbranchpriorities.restype  = ctypes.c_int
__library__.MSK_XX_readbranchpriorities.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_writebranchpriorities.restype  = ctypes.c_int
__library__.MSK_XX_writebranchpriorities.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_writeparamfile.restype  = ctypes.c_int
__library__.MSK_XX_writeparamfile.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_getinfeasiblesubproblem.restype  = ctypes.c_int
__library__.MSK_XX_getinfeasiblesubproblem.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_void_p)]
__library__.MSK_XX_writesolution.restype  = ctypes.c_int
__library__.MSK_XX_writesolution.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p]
__library__.MSK_XX_primalsensitivity.restype  = ctypes.c_int
__library__.MSK_XX_primalsensitivity.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_sensitivityreport.restype  = ctypes.c_int
__library__.MSK_XX_sensitivityreport.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_dualsensitivity.restype  = ctypes.c_int
__library__.MSK_XX_dualsensitivity.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_checkconvexity.restype  = ctypes.c_int
__library__.MSK_XX_checkconvexity.argtypes = [ctypes.c_void_p]
__library__.MSK_XX_checkoutlicense.restype  = ctypes.c_int
__library__.MSK_XX_checkoutlicense.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_checkinlicense.restype  = ctypes.c_int
__library__.MSK_XX_checkinlicense.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_echointro.restype  = ctypes.c_int
__library__.MSK_XX_echointro.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_getcodedesc.restype  = ctypes.c_int
__library__.MSK_XX_getcodedesc.argtypes = [ctypes.c_int32,ctypes.c_char_p,ctypes.c_char_p]
__library__.MSK_XX_getversion.restype  = ctypes.c_int
__library__.MSK_XX_getversion.argtypes = [ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_linkfiletoenvstream.restype  = ctypes.c_int
__library__.MSK_XX_linkfiletoenvstream.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_char_p,ctypes.c_int32]
__library__.MSK_XX_putdllpath.restype  = ctypes.c_int
__library__.MSK_XX_putdllpath.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_putlicensedebug.restype  = ctypes.c_int
__library__.MSK_XX_putlicensedebug.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_putlicensecode.restype  = ctypes.c_int
__library__.MSK_XX_putlicensecode.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
__library__.MSK_XX_putlicensewait.restype  = ctypes.c_int
__library__.MSK_XX_putlicensewait.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_putlicensepath.restype  = ctypes.c_int
__library__.MSK_XX_putlicensepath.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
__library__.MSK_XX_putkeepdlls.restype  = ctypes.c_int
__library__.MSK_XX_putkeepdlls.argtypes = [ctypes.c_void_p,ctypes.c_int32]
__library__.MSK_XX_axpy.restype  = ctypes.c_int
__library__.MSK_XX_axpy.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_double,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_dot.restype  = ctypes.c_int
__library__.MSK_XX_dot.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_gemv.restype  = ctypes.c_int
__library__.MSK_XX_gemv.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_gemm.restype  = ctypes.c_int
__library__.MSK_XX_gemm.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_syrk.restype  = ctypes.c_int
__library__.MSK_XX_syrk.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_potrf.restype  = ctypes.c_int
__library__.MSK_XX_potrf.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_syeig.restype  = ctypes.c_int
__library__.MSK_XX_syeig.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_syevd.restype  = ctypes.c_int
__library__.MSK_XX_syevd.argtypes = [ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
__library__.MSK_XX_licensecleanup.restype  = ctypes.c_int
__library__.MSK_XX_licensecleanup.argtypes = []

__scopt__ = None
__scopt_guard__ = threading.Lock()
def __load_scopt__():
    global __scopt__
    __scopt_guard__.acquire()
    try:
        if __scopt__ is None:
            mskmajorver = ctypes.c_int()
            mskminorver = ctypes.c_int()
            mskbuildver = ctypes.c_int()
            mskrevision = ctypes.c_int()
            __library__.MSK_XX_getversion(ctypes.byref(mskmajorver),
                                          ctypes.byref(mskminorver),
                                          ctypes.byref(mskbuildver),
                                          ctypes.byref(mskrevision))
            if __libloadpath is not None:
                __scopt__ = __library_factory__(os.path.join(__libloadpath,__makelibname('mosekscopt%d_%d' % (mskmajorver.value,mskminorver.value))))
            else:
                __scopt__ = __library_factory__(__makelibname('mosekscopt%d_%d' % (mskmajorver.value,mskminorver.value)))
            __scopt__.MSK_scbegin.restype  = ctypes.c_int
            __scopt__.MSK_scbegin.argtypes = [ ctypes.c_void_p, # task
                                               ctypes.c_int, # numopro
                                               ctypes.POINTER(ctypes.c_int), # opro
                                               ctypes.POINTER(ctypes.c_int), # oprjo
                                               ctypes.POINTER(ctypes.c_double), # oprfo
                                               ctypes.POINTER(ctypes.c_double), # oprgo
                                               ctypes.POINTER(ctypes.c_double), # oprho
                                               ctypes.c_int, # numoprc
                                               ctypes.POINTER(ctypes.c_int), # oprc
                                               ctypes.POINTER(ctypes.c_int), # opric
                                               ctypes.POINTER(ctypes.c_int), # oprjc
                                               ctypes.POINTER(ctypes.c_double), # oprfc
                                               ctypes.POINTER(ctypes.c_double), # oprgc
                                               ctypes.POINTER(ctypes.c_double), # oprhc
                                               ctypes.POINTER(ctypes.c_void_p),  # schandle
                                               ]
            __scopt__.MSK_scend.restype  = ctypes.c_int
            __scopt__.MSK_scend.argtypes = [ ctypes.c_void_p, # task
                                             ctypes.c_void_p, # schandle
                                             ]
            __scopt__.MSK_scattach.restype  = ctypes.c_int
            __scopt__.MSK_scattach.argtypes = [ ctypes.c_void_p, # task
                                                ctypes.c_void_p, # schandle
                                                ]
            
            __scopt__.MSK_scdetach.restype  = ctypes.c_int
            __scopt__.MSK_scdetach.argtypes = [ ctypes.c_void_p, # task
                                                ]
    finally:
        __scopt_guard__.release()

scopr = Enum("scopr", ["ent","exp","log","pow"], [ 0, 1, 2, 3 ])
solveform = Enum("solveform", ["dual","free","primal"], [2,0,1])
problemitem = Enum("problemitem", ["con","cone","var"], [1,2,0])
accmode = Enum("accmode", ["con","var"], [1,0])
sensitivitytype = Enum("sensitivitytype", ["basis","optimal_partition"], [0,1])
uplo = Enum("uplo", ["lo","up"], [0,1])
intpnthotstart = Enum("intpnthotstart", ["dual","none","primal","primal_dual"], [2,0,1,3])
sparam = Enum("sparam", ["bas_sol_file_name","data_file_name","debug_file_name","feasrepair_name_prefix","feasrepair_name_separator","feasrepair_name_wsumviol","int_sol_file_name","itr_sol_file_name","mio_debug_string","param_comment_sign","param_read_file_name","param_write_file_name","read_mps_bou_name","read_mps_obj_name","read_mps_ran_name","read_mps_rhs_name","sensitivity_file_name","sensitivity_res_file_name","sol_filter_xc_low","sol_filter_xc_upr","sol_filter_xx_low","sol_filter_xx_upr","stat_file_name","stat_key","stat_name","write_lp_gen_var_name"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25])
iparam = Enum("iparam", ["alloc_add_qnz","ana_sol_basis","ana_sol_print_violated","auto_sort_a_before_opt","auto_update_sol_info","basis_solve_use_plus_one","bi_clean_optimizer","bi_ignore_max_iter","bi_ignore_num_error","bi_max_iterations","cache_license","check_convexity","compress_statfile","concurrent_num_optimizers","concurrent_priority_dual_simplex","concurrent_priority_free_simplex","concurrent_priority_intpnt","concurrent_priority_primal_simplex","feasrepair_optimize","infeas_generic_names","infeas_prefer_primal","infeas_report_auto","infeas_report_level","intpnt_basis","intpnt_diff_step","intpnt_factor_debug_lvl","intpnt_factor_method","intpnt_hotstart","intpnt_max_iterations","intpnt_max_num_cor","intpnt_max_num_refinement_steps","intpnt_off_col_trh","intpnt_order_method","intpnt_regularization_use","intpnt_scaling","intpnt_solve_form","intpnt_starting_point","lic_trh_expiry_wrn","license_debug","license_pause_time","license_suppress_expire_wrns","license_wait","log","log_bi","log_bi_freq","log_check_convexity","log_concurrent","log_cut_second_opt","log_expand","log_factor","log_feas_repair","log_file","log_head","log_infeas_ana","log_intpnt","log_mio","log_mio_freq","log_nonconvex","log_optimizer","log_order","log_param","log_presolve","log_response","log_sensitivity","log_sensitivity_opt","log_sim","log_sim_freq","log_sim_minor","log_sim_network_freq","log_storage","max_num_warnings","mio_branch_dir","mio_branch_priorities_use","mio_construct_sol","mio_cont_sol","mio_cut_cg","mio_cut_cmir","mio_cut_level_root","mio_cut_level_tree","mio_feaspump_level","mio_heuristic_level","mio_hotstart","mio_keep_basis","mio_local_branch_number","mio_max_num_branches","mio_max_num_relaxs","mio_max_num_solutions","mio_mode","mio_mt_user_cb","mio_node_optimizer","mio_node_selection","mio_optimizer_mode","mio_presolve_aggregate","mio_presolve_probing","mio_presolve_use","mio_probing_level","mio_rins_max_nodes","mio_root_optimizer","mio_strong_branch","mio_use_multithreaded_optimizer","mt_spincount","nonconvex_max_iterations","num_threads","opf_max_terms_per_line","opf_write_header","opf_write_hints","opf_write_parameters","opf_write_problem","opf_write_sol_bas","opf_write_sol_itg","opf_write_sol_itr","opf_write_solutions","optimizer","param_read_case_name","param_read_ign_error","presolve_elim_fill","presolve_eliminator_max_num_tries","presolve_eliminator_use","presolve_level","presolve_lindep_abs_work_trh","presolve_lindep_rel_work_trh","presolve_lindep_use","presolve_max_num_reductions","presolve_use","primal_repair_optimizer","qo_separable_reformulation","read_anz","read_con","read_cone","read_data_compressed","read_data_format","read_debug","read_keep_free_con","read_lp_drop_new_vars_in_bou","read_lp_quoted_names","read_mps_format","read_mps_keep_int","read_mps_obj_sense","read_mps_relax","read_mps_width","read_qnz","read_task_ignore_param","read_var","sensitivity_all","sensitivity_optimizer","sensitivity_type","sim_basis_factor_use","sim_degen","sim_dual_crash","sim_dual_phaseone_method","sim_dual_restrict_selection","sim_dual_selection","sim_exploit_dupvec","sim_hotstart","sim_hotstart_lu","sim_integer","sim_max_iterations","sim_max_num_setbacks","sim_non_singular","sim_primal_crash","sim_primal_phaseone_method","sim_primal_restrict_selection","sim_primal_selection","sim_refactor_freq","sim_reformulation","sim_save_lu","sim_scaling","sim_scaling_method","sim_solve_form","sim_stability_priority","sim_switch_optimizer","sol_filter_keep_basic","sol_filter_keep_ranged","sol_read_name_width","sol_read_width","solution_callback","timing_level","warning_level","write_bas_constraints","write_bas_head","write_bas_variables","write_data_compressed","write_data_format","write_data_param","write_free_con","write_generic_names","write_generic_names_io","write_ignore_incompatible_conic_items","write_ignore_incompatible_items","write_ignore_incompatible_nl_items","write_ignore_incompatible_psd_items","write_int_constraints","write_int_head","write_int_variables","write_lp_line_width","write_lp_quoted_names","write_lp_strict_format","write_lp_terms_per_line","write_mps_int","write_precision","write_sol_barvariables","write_sol_constraints","write_sol_head","write_sol_ignore_invalid_names","write_sol_variables","write_task_inc_sol","write_xml_mode"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206])
solsta = Enum("solsta", ["dual_feas","dual_infeas_cer","integer_optimal","near_dual_feas","near_dual_infeas_cer","near_integer_optimal","near_optimal","near_prim_and_dual_feas","near_prim_feas","near_prim_infeas_cer","optimal","prim_and_dual_feas","prim_feas","prim_infeas_cer","unknown"], [3,6,14,10,13,15,8,11,9,12,1,4,2,5,0])
objsense = Enum("objsense", ["maximize","minimize"], [1,0])
solitem = Enum("solitem", ["slc","slx","snx","suc","sux","xc","xx","y"], [3,5,7,4,6,0,1,2])
boundkey = Enum("boundkey", ["fr","fx","lo","ra","up"], [3,2,0,4,1])
basindtype = Enum("basindtype", ["always","if_feasible","never","no_error","reservered"], [1,3,0,2,4])
branchdir = Enum("branchdir", ["down","free","up"], [2,0,1])
liinfitem = Enum("liinfitem", ["bi_clean_dual_deg_iter","bi_clean_dual_iter","bi_clean_primal_deg_iter","bi_clean_primal_dual_deg_iter","bi_clean_primal_dual_iter","bi_clean_primal_dual_sub_iter","bi_clean_primal_iter","bi_dual_iter","bi_primal_iter","intpnt_factor_num_nz","mio_intpnt_iter","mio_simplex_iter","rd_numanz","rd_numqnz"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13])
streamtype = Enum("streamtype", ["err","log","msg","wrn"], [2,0,1,3])
simhotstart = Enum("simhotstart", ["free","none","status_keys"], [1,0,2])
callbackcode = Enum("callbackcode", ["begin_bi","begin_concurrent","begin_conic","begin_dual_bi","begin_dual_sensitivity","begin_dual_setup_bi","begin_dual_simplex","begin_dual_simplex_bi","begin_full_convexity_check","begin_infeas_ana","begin_intpnt","begin_license_wait","begin_mio","begin_network_dual_simplex","begin_network_primal_simplex","begin_network_simplex","begin_nonconvex","begin_optimizer","begin_presolve","begin_primal_bi","begin_primal_dual_simplex","begin_primal_dual_simplex_bi","begin_primal_repair","begin_primal_sensitivity","begin_primal_setup_bi","begin_primal_simplex","begin_primal_simplex_bi","begin_qcqo_reformulate","begin_read","begin_simplex","begin_simplex_bi","begin_simplex_network_detect","begin_write","conic","dual_simplex","end_bi","end_concurrent","end_conic","end_dual_bi","end_dual_sensitivity","end_dual_setup_bi","end_dual_simplex","end_dual_simplex_bi","end_full_convexity_check","end_infeas_ana","end_intpnt","end_license_wait","end_mio","end_network_dual_simplex","end_network_primal_simplex","end_network_simplex","end_nonconvex","end_optimizer","end_presolve","end_primal_bi","end_primal_dual_simplex","end_primal_dual_simplex_bi","end_primal_repair","end_primal_sensitivity","end_primal_setup_bi","end_primal_simplex","end_primal_simplex_bi","end_qcqo_reformulate","end_read","end_simplex","end_simplex_bi","end_simplex_network_detect","end_write","im_bi","im_conic","im_dual_bi","im_dual_sensivity","im_dual_simplex","im_full_convexity_check","im_intpnt","im_license_wait","im_lu","im_mio","im_mio_dual_simplex","im_mio_intpnt","im_mio_presolve","im_mio_primal_simplex","im_network_dual_simplex","im_network_primal_simplex","im_nonconvex","im_order","im_presolve","im_primal_bi","im_primal_dual_simplex","im_primal_sensivity","im_primal_simplex","im_qo_reformulate","im_read","im_simplex","im_simplex_bi","intpnt","new_int_mio","noncovex","primal_simplex","read_opf","read_opf_section","update_dual_bi","update_dual_simplex","update_dual_simplex_bi","update_network_dual_simplex","update_network_primal_simplex","update_nonconvex","update_presolve","update_primal_bi","update_primal_dual_simplex","update_primal_dual_simplex_bi","update_primal_simplex","update_primal_simplex_bi","write_opf"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113])
symmattype = Enum("symmattype", ["sparse"], [0])
feature = Enum("feature", ["ptom","pton","ptox","pts"], [2,1,3,0])
mark = Enum("mark", ["lo","up"], [0,1])
conetype = Enum("conetype", ["quad","rquad"], [0,1])
feasrepairtype = Enum("feasrepairtype", ["optimize_combined","optimize_none","optimize_penalty"], [2,0,1])
iomode = Enum("iomode", ["read","readwrite","write"], [0,2,1])
simseltype = Enum("simseltype", ["ase","devex","free","full","partial","se"], [2,3,0,1,5,4])
msgkey = Enum("msgkey", ["mps_selected","reading_file","writing_file"], [1100,1000,1001])
miomode = Enum("miomode", ["ignored","lazy","satisfied"], [0,2,1])
dinfitem = Enum("dinfitem", ["bi_clean_dual_time","bi_clean_primal_dual_time","bi_clean_primal_time","bi_clean_time","bi_dual_time","bi_primal_time","bi_time","concurrent_time","intpnt_dual_feas","intpnt_dual_obj","intpnt_factor_num_flops","intpnt_opt_status","intpnt_order_time","intpnt_primal_feas","intpnt_primal_obj","intpnt_time","mio_cg_seperation_time","mio_cmir_seperation_time","mio_construct_solution_obj","mio_dual_bound_after_presolve","mio_heuristic_time","mio_obj_abs_gap","mio_obj_bound","mio_obj_int","mio_obj_rel_gap","mio_optimizer_time","mio_probing_time","mio_root_cutgen_time","mio_root_optimizer_time","mio_root_presolve_time","mio_time","mio_user_obj_cut","optimizer_time","presolve_eli_time","presolve_lindep_time","presolve_time","primal_repair_penalty_obj","qcqo_reformulate_time","rd_time","sim_dual_time","sim_feas","sim_network_dual_time","sim_network_primal_time","sim_network_time","sim_obj","sim_primal_dual_time","sim_primal_time","sim_time","sol_bas_dual_obj","sol_bas_dviolcon","sol_bas_dviolvar","sol_bas_primal_obj","sol_bas_pviolcon","sol_bas_pviolvar","sol_itg_primal_obj","sol_itg_pviolbarvar","sol_itg_pviolcon","sol_itg_pviolcones","sol_itg_pviolitg","sol_itg_pviolvar","sol_itr_dual_obj","sol_itr_dviolbarvar","sol_itr_dviolcon","sol_itr_dviolcones","sol_itr_dviolvar","sol_itr_primal_obj","sol_itr_pviolbarvar","sol_itr_pviolcon","sol_itr_pviolcones","sol_itr_pviolvar"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69])
parametertype = Enum("parametertype", ["dou_type","int_type","invalid_type","str_type"], [1,2,0,3])
rescodetype = Enum("rescodetype", ["err","ok","trm","unk","wrn"], [3,0,2,4,1])
prosta = Enum("prosta", ["dual_feas","dual_infeas","ill_posed","near_dual_feas","near_prim_and_dual_feas","near_prim_feas","prim_and_dual_feas","prim_and_dual_infeas","prim_feas","prim_infeas","prim_infeas_or_unbounded","unknown"], [3,5,7,10,8,9,1,6,2,4,11,0])
scalingtype = Enum("scalingtype", ["aggressive","free","moderate","none"], [3,0,2,1])
rescode = Enum("rescode", ["err_ad_invalid_codelist","err_ad_invalid_operand","err_ad_invalid_operator","err_ad_missing_operand","err_ad_missing_return","err_api_array_too_small","err_api_cb_connect","err_api_fatal_error","err_api_internal","err_arg_is_too_large","err_arg_is_too_small","err_argument_dimension","err_argument_is_too_large","err_argument_lenneq","err_argument_perm_array","err_argument_type","err_bar_var_dim","err_basis","err_basis_factor","err_basis_singular","err_blank_name","err_cannot_clone_nl","err_cannot_handle_nl","err_cbf_duplicate_acoord","err_cbf_duplicate_bcoord","err_cbf_duplicate_con","err_cbf_duplicate_int","err_cbf_duplicate_obj","err_cbf_duplicate_objacoord","err_cbf_duplicate_var","err_cbf_invalid_con_type","err_cbf_invalid_domain_dimension","err_cbf_invalid_int_index","err_cbf_invalid_var_type","err_cbf_no_variables","err_cbf_no_version_specified","err_cbf_obj_sense","err_cbf_parse","err_cbf_syntax","err_cbf_too_few_constraints","err_cbf_too_few_ints","err_cbf_too_few_variables","err_cbf_too_many_constraints","err_cbf_too_many_ints","err_cbf_too_many_variables","err_cbf_unsupported","err_con_q_not_nsd","err_con_q_not_psd","err_concurrent_optimizer","err_cone_index","err_cone_overlap","err_cone_overlap_append","err_cone_rep_var","err_cone_size","err_cone_type","err_cone_type_str","err_data_file_ext","err_dup_name","err_duplicate_barvariable_names","err_duplicate_cone_names","err_duplicate_constraint_names","err_duplicate_variable_names","err_end_of_file","err_factor","err_feasrepair_cannot_relax","err_feasrepair_inconsistent_bound","err_feasrepair_solving_relaxed","err_file_license","err_file_open","err_file_read","err_file_write","err_first","err_firsti","err_firstj","err_fixed_bound_values","err_flexlm","err_global_inv_conic_problem","err_huge_aij","err_huge_c","err_identical_tasks","err_in_argument","err_index","err_index_arr_is_too_large","err_index_arr_is_too_small","err_index_is_too_large","err_index_is_too_small","err_inf_dou_index","err_inf_dou_name","err_inf_int_index","err_inf_int_name","err_inf_lint_index","err_inf_lint_name","err_inf_type","err_infeas_undefined","err_infinite_bound","err_int64_to_int32_cast","err_internal","err_internal_test_failed","err_inv_aptre","err_inv_bk","err_inv_bkc","err_inv_bkx","err_inv_cone_type","err_inv_cone_type_str","err_inv_conic_problem","err_inv_marki","err_inv_markj","err_inv_name_item","err_inv_numi","err_inv_numj","err_inv_optimizer","err_inv_problem","err_inv_qcon_subi","err_inv_qcon_subj","err_inv_qcon_subk","err_inv_qcon_val","err_inv_qobj_subi","err_inv_qobj_subj","err_inv_qobj_val","err_inv_sk","err_inv_sk_str","err_inv_skc","err_inv_skn","err_inv_skx","err_inv_var_type","err_invalid_accmode","err_invalid_aij","err_invalid_ampl_stub","err_invalid_barvar_name","err_invalid_branch_direction","err_invalid_branch_priority","err_invalid_compression","err_invalid_con_name","err_invalid_cone_name","err_invalid_file_format_for_cones","err_invalid_file_format_for_general_nl","err_invalid_file_format_for_sym_mat","err_invalid_file_name","err_invalid_format_type","err_invalid_idx","err_invalid_iomode","err_invalid_max_num","err_invalid_name_in_sol_file","err_invalid_network_problem","err_invalid_obj_name","err_invalid_objective_sense","err_invalid_problem_type","err_invalid_sol_file_name","err_invalid_stream","err_invalid_surplus","err_invalid_sym_mat_dim","err_invalid_task","err_invalid_utf8","err_invalid_var_name","err_invalid_wchar","err_invalid_whichsol","err_last","err_lasti","err_lastj","err_lau_arg_k","err_lau_arg_m","err_lau_arg_n","err_lau_arg_trans","err_lau_arg_transa","err_lau_arg_transb","err_lau_arg_uplo","err_lau_singular_matrix","err_lau_unknown","err_license","err_license_cannot_allocate","err_license_cannot_connect","err_license_expired","err_license_feature","err_license_invalid_hostid","err_license_max","err_license_moseklm_daemon","err_license_no_server_line","err_license_no_server_support","err_license_server","err_license_server_version","err_license_version","err_link_file_dll","err_living_tasks","err_lower_bound_is_a_nan","err_lp_dup_slack_name","err_lp_empty","err_lp_file_format","err_lp_format","err_lp_free_constraint","err_lp_incompatible","err_lp_invalid_con_name","err_lp_invalid_var_name","err_lp_write_conic_problem","err_lp_write_geco_problem","err_lu_max_num_tries","err_max_len_is_too_small","err_maxnumbarvar","err_maxnumcon","err_maxnumcone","err_maxnumqnz","err_maxnumvar","err_mbt_incompatible","err_mbt_invalid","err_mio_internal","err_mio_invalid_node_optimizer","err_mio_invalid_root_optimizer","err_mio_no_optimizer","err_mio_not_loaded","err_missing_license_file","err_mixed_problem","err_mps_cone_overlap","err_mps_cone_repeat","err_mps_cone_type","err_mps_duplicate_q_element","err_mps_file","err_mps_inv_bound_key","err_mps_inv_con_key","err_mps_inv_field","err_mps_inv_marker","err_mps_inv_sec_name","err_mps_inv_sec_order","err_mps_invalid_obj_name","err_mps_invalid_objsense","err_mps_mul_con_name","err_mps_mul_csec","err_mps_mul_qobj","err_mps_mul_qsec","err_mps_no_objective","err_mps_non_symmetric_q","err_mps_null_con_name","err_mps_null_var_name","err_mps_splitted_var","err_mps_tab_in_field2","err_mps_tab_in_field3","err_mps_tab_in_field5","err_mps_undef_con_name","err_mps_undef_var_name","err_mul_a_element","err_name_is_null","err_name_max_len","err_nan_in_blc","err_nan_in_blx","err_nan_in_buc","err_nan_in_bux","err_nan_in_c","err_nan_in_double_data","err_negative_append","err_negative_surplus","err_newer_dll","err_no_bars_for_solution","err_no_barx_for_solution","err_no_basis_sol","err_no_dual_for_itg_sol","err_no_dual_infeas_cer","err_no_dual_info_for_itg_sol","err_no_init_env","err_no_optimizer_var_type","err_no_primal_infeas_cer","err_no_snx_for_bas_sol","err_no_solution_in_callback","err_non_unique_array","err_nonconvex","err_nonlinear_equality","err_nonlinear_functions_not_allowed","err_nonlinear_ranged","err_nr_arguments","err_null_env","err_null_pointer","err_null_task","err_numconlim","err_numvarlim","err_obj_q_not_nsd","err_obj_q_not_psd","err_objective_range","err_older_dll","err_open_dl","err_opf_format","err_opf_new_variable","err_opf_premature_eof","err_optimizer_license","err_ord_invalid","err_ord_invalid_branch_dir","err_overflow","err_param_index","err_param_is_too_large","err_param_is_too_small","err_param_name","err_param_name_dou","err_param_name_int","err_param_name_str","err_param_type","err_param_value_str","err_platform_not_licensed","err_postsolve","err_pro_item","err_prob_license","err_qcon_subi_too_large","err_qcon_subi_too_small","err_qcon_upper_triangle","err_qobj_upper_triangle","err_read_format","err_read_lp_missing_end_tag","err_read_lp_nonexisting_name","err_remove_cone_variable","err_repair_invalid_problem","err_repair_optimization_failed","err_sen_bound_invalid_lo","err_sen_bound_invalid_up","err_sen_format","err_sen_index_invalid","err_sen_index_range","err_sen_invalid_regexp","err_sen_numerical","err_sen_solution_status","err_sen_undef_name","err_sen_unhandled_problem_type","err_size_license","err_size_license_con","err_size_license_intvar","err_size_license_numcores","err_size_license_var","err_sol_file_invalid_number","err_solitem","err_solver_probtype","err_space","err_space_leaking","err_space_no_info","err_sym_mat_duplicate","err_sym_mat_invalid_col_index","err_sym_mat_invalid_row_index","err_sym_mat_invalid_value","err_sym_mat_not_lower_tringular","err_task_incompatible","err_task_invalid","err_thread_cond_init","err_thread_create","err_thread_mutex_init","err_thread_mutex_lock","err_thread_mutex_unlock","err_toconic_conversion_fail","err_too_many_concurrent_tasks","err_too_small_max_num_nz","err_too_small_maxnumanz","err_unb_step_size","err_undef_solution","err_undefined_objective_sense","err_unhandled_solution_status","err_unknown","err_upper_bound_is_a_nan","err_upper_triangle","err_user_func_ret","err_user_func_ret_data","err_user_nlo_eval","err_user_nlo_eval_hessubi","err_user_nlo_eval_hessubj","err_user_nlo_func","err_whichitem_not_allowed","err_whichsol","err_write_lp_format","err_write_lp_non_unique_name","err_write_mps_invalid_name","err_write_opf_invalid_var_name","err_writing_file","err_xml_invalid_problem_type","err_y_is_undefined","ok","trm_internal","trm_internal_stop","trm_max_iterations","trm_max_num_setbacks","trm_max_time","trm_mio_near_abs_gap","trm_mio_near_rel_gap","trm_mio_num_branches","trm_mio_num_relaxs","trm_num_max_num_int_solutions","trm_numerical_problem","trm_objective_range","trm_stall","trm_user_callback","wrn_ana_almost_int_bounds","wrn_ana_c_zero","wrn_ana_close_bounds","wrn_ana_empty_cols","wrn_ana_large_bounds","wrn_construct_invalid_sol_itg","wrn_construct_no_sol_itg","wrn_construct_solution_infeas","wrn_dropped_nz_qobj","wrn_duplicate_barvariable_names","wrn_duplicate_cone_names","wrn_duplicate_constraint_names","wrn_duplicate_variable_names","wrn_eliminator_space","wrn_empty_name","wrn_ignore_integer","wrn_incomplete_linear_dependency_check","wrn_large_aij","wrn_large_bound","wrn_large_cj","wrn_large_con_fx","wrn_large_lo_bound","wrn_large_up_bound","wrn_license_expire","wrn_license_feature_expire","wrn_license_server","wrn_lp_drop_variable","wrn_lp_old_quad_format","wrn_mio_infeasible_final","wrn_mps_split_bou_vector","wrn_mps_split_ran_vector","wrn_mps_split_rhs_vector","wrn_name_max_len","wrn_no_dualizer","wrn_no_global_optimizer","wrn_no_nonlinear_function_write","wrn_nz_in_upr_tri","wrn_open_param_file","wrn_param_ignored_cmio","wrn_param_name_dou","wrn_param_name_int","wrn_param_name_str","wrn_param_str_value","wrn_presolve_outofspace","wrn_quad_cones_with_root_fixed_at_zero","wrn_rquad_cones_with_root_fixed_at_zero","wrn_sol_file_ignored_con","wrn_sol_file_ignored_var","wrn_sol_filter","wrn_spar_max_len","wrn_too_few_basis_vars","wrn_too_many_basis_vars","wrn_too_many_threads_concurrent","wrn_undef_sol_file_name","wrn_using_generic_names","wrn_write_changed_names","wrn_write_discarded_cfix","wrn_zero_aij","wrn_zeros_in_sparse_col","wrn_zeros_in_sparse_row"], [3102,3104,3103,3105,3106,3001,3002,3005,3999,1227,1226,1201,5005,1197,1299,1198,3920,1266,1610,1615,1070,2505,2506,7116,7115,7108,7110,7107,7114,7109,7112,7113,7121,7111,7102,7105,7101,7100,7106,7118,7119,7117,7103,7120,7104,7122,1294,1293,3059,1300,1302,1307,1303,1301,1305,1306,1055,1071,4502,4503,4500,4501,1059,1650,1700,1702,1701,1007,1052,1053,1054,1261,1285,1287,1425,1014,1503,1380,1375,3101,1200,1235,1222,1221,1204,1203,1219,1230,1220,1231,1225,1234,1232,3910,1400,3800,3000,3500,1253,1255,1256,1257,1272,1271,1502,2501,2502,1280,2503,2504,1550,1500,1405,1406,1404,1407,1401,1402,1403,1270,1269,1267,1274,1268,1258,2520,1473,3700,1079,3200,3201,1800,1076,1078,4005,4010,4000,1056,1283,1246,1801,1247,1170,1504,1075,1445,6000,1057,1062,1275,3950,1064,2900,1077,2901,1228,1262,1286,1288,7004,7002,7003,7008,7005,7006,7007,7000,7001,1000,1020,1021,1001,1018,1025,1016,1017,1028,1027,1015,1026,1002,1040,1066,1390,1152,1151,1157,1160,1155,1150,1171,1154,1163,1164,2800,1289,1242,1240,1304,1243,1241,2550,2551,5010,7131,7130,1551,1553,1008,1501,1118,1119,1117,1121,1100,1108,1107,1101,1102,1109,1115,1128,1122,1112,1116,1114,1113,1110,1120,1103,1104,1111,1125,1126,1127,1105,1106,1254,1760,1750,1461,1471,1462,1472,1470,1450,1264,1263,1036,3916,3915,1600,2950,2001,3300,1063,1552,2000,2953,2500,5000,1291,1290,1428,1292,1199,1060,1065,1061,1250,1251,1296,1295,1260,1035,1030,1168,1169,1172,1013,1131,1130,1590,1210,1215,1216,1205,1206,1207,1208,1218,1217,1019,1580,1281,1006,1409,1408,1417,1415,1090,1159,1162,1310,1710,1711,3054,3053,3050,3055,3052,3056,3058,3057,3051,3080,1005,1010,1012,3900,1011,1350,1237,1259,1051,1080,1081,3944,3941,3940,3943,3942,2560,2561,1049,1048,1045,1046,1047,7200,3090,1245,1252,3100,1265,1446,6010,1050,1391,6020,1430,1431,1433,1440,1441,1432,1238,1236,1158,1161,1153,1156,1166,3600,1449,0,10030,10031,10000,10020,10001,10004,10003,10009,10008,10015,10025,10002,10006,10007,904,901,903,902,900,807,810,805,201,852,853,850,851,801,502,250,800,62,51,57,54,52,53,500,505,501,85,80,270,72,71,70,65,950,251,450,200,50,516,510,511,512,515,802,930,931,351,352,300,66,400,405,750,350,503,803,804,63,710,705])
mionodeseltype = Enum("mionodeseltype", ["best","first","free","hybrid","pseudo","worst"], [2,1,0,4,5,3])
transpose = Enum("transpose", ["no","yes"], [0,1])
onoffkey = Enum("onoffkey", ["off","on"], [0,1])
simdegen = Enum("simdegen", ["aggressive","free","minimum","moderate","none"], [2,1,4,3,0])
dataformat = Enum("dataformat", ["cb","extension","free_mps","lp","mps","op","task","xml"], [7,0,5,2,1,3,6,4])
orderingtype = Enum("orderingtype", ["appminloc","experimental","force_graphpar","free","none","try_graphpar"], [1,2,4,0,5,3])
problemtype = Enum("problemtype", ["conic","geco","lo","mixed","qcqo","qo"], [4,3,0,5,2,1])
inftype = Enum("inftype", ["dou_type","int_type","lint_type"], [0,1,2])
dparam = Enum("dparam", ["ana_sol_infeas_tol","basis_rel_tol_s","basis_tol_s","basis_tol_x","check_convexity_rel_tol","data_tol_aij","data_tol_aij_huge","data_tol_aij_large","data_tol_bound_inf","data_tol_bound_wrn","data_tol_c_huge","data_tol_cj_large","data_tol_qij","data_tol_x","feasrepair_tol","intpnt_co_tol_dfeas","intpnt_co_tol_infeas","intpnt_co_tol_mu_red","intpnt_co_tol_near_rel","intpnt_co_tol_pfeas","intpnt_co_tol_rel_gap","intpnt_nl_merit_bal","intpnt_nl_tol_dfeas","intpnt_nl_tol_mu_red","intpnt_nl_tol_near_rel","intpnt_nl_tol_pfeas","intpnt_nl_tol_rel_gap","intpnt_nl_tol_rel_step","intpnt_tol_dfeas","intpnt_tol_dsafe","intpnt_tol_infeas","intpnt_tol_mu_red","intpnt_tol_path","intpnt_tol_pfeas","intpnt_tol_psafe","intpnt_tol_rel_gap","intpnt_tol_rel_step","intpnt_tol_step_size","lower_obj_cut","lower_obj_cut_finite_trh","mio_disable_term_time","mio_heuristic_time","mio_max_time","mio_max_time_aprx_opt","mio_near_tol_abs_gap","mio_near_tol_rel_gap","mio_rel_add_cut_limited","mio_rel_gap_const","mio_tol_abs_gap","mio_tol_abs_relax_int","mio_tol_feas","mio_tol_max_cut_frac_rhs","mio_tol_min_cut_frac_rhs","mio_tol_rel_dual_bound_improvement","mio_tol_rel_gap","mio_tol_rel_relax_int","mio_tol_x","nonconvex_tol_feas","nonconvex_tol_opt","optimizer_max_time","presolve_tol_abs_lindep","presolve_tol_aij","presolve_tol_rel_lindep","presolve_tol_s","presolve_tol_x","qcqo_reformulate_rel_drop_tol","sim_lu_tol_rel_piv","simplex_abs_tol_piv","upper_obj_cut","upper_obj_cut_finite_trh"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69])
simdupvec = Enum("simdupvec", ["free","off","on"], [2,0,1])
compresstype = Enum("compresstype", ["free","gzip","none"], [1,2,0])
nametype = Enum("nametype", ["gen","lp","mps"], [0,2,1])
mpsformat = Enum("mpsformat", ["free","relaxed","strict"], [2,1,0])
variabletype = Enum("variabletype", ["type_cont","type_int"], [0,1])
checkconvexitytype = Enum("checkconvexitytype", ["full","none","simple"], [2,0,1])
language = Enum("language", ["dan","eng"], [1,0])
startpointtype = Enum("startpointtype", ["constant","free","guess","satisfy_bounds"], [2,0,1,3])
soltype = Enum("soltype", ["bas","itg","itr"], [1,2,0])
scalingmethod = Enum("scalingmethod", ["free","pow2"], [1,0])
value = Enum("value", ["license_buffer_length","max_str_len"], [20,1024])
stakey = Enum("stakey", ["bas","fix","inf","low","supbas","unk","upr"], [1,5,6,3,2,0,4])
simreform = Enum("simreform", ["aggressive","free","off","on"], [3,2,0,1])
iinfitem = Enum("iinfitem", ["ana_pro_num_con","ana_pro_num_con_eq","ana_pro_num_con_fr","ana_pro_num_con_lo","ana_pro_num_con_ra","ana_pro_num_con_up","ana_pro_num_var","ana_pro_num_var_bin","ana_pro_num_var_cont","ana_pro_num_var_eq","ana_pro_num_var_fr","ana_pro_num_var_int","ana_pro_num_var_lo","ana_pro_num_var_ra","ana_pro_num_var_up","concurrent_fastest_optimizer","intpnt_factor_dim_dense","intpnt_iter","intpnt_num_threads","intpnt_solve_dual","mio_construct_num_roundings","mio_construct_solution","mio_initial_solution","mio_num_active_nodes","mio_num_basis_cuts","mio_num_branch","mio_num_cardgub_cuts","mio_num_clique_cuts","mio_num_coef_redc_cuts","mio_num_contra_cuts","mio_num_disagg_cuts","mio_num_flow_cover_cuts","mio_num_gcd_cuts","mio_num_gomory_cuts","mio_num_gub_cover_cuts","mio_num_int_solutions","mio_num_knapsur_cover_cuts","mio_num_lattice_cuts","mio_num_lift_cuts","mio_num_obj_cuts","mio_num_plan_loc_cuts","mio_num_relax","mio_numcon","mio_numint","mio_numvar","mio_obj_bound_defined","mio_total_num_cuts","mio_user_obj_cut","opt_numcon","opt_numvar","optimize_response","rd_numbarvar","rd_numcon","rd_numcone","rd_numintvar","rd_numq","rd_numvar","rd_protype","sim_dual_deg_iter","sim_dual_hotstart","sim_dual_hotstart_lu","sim_dual_inf_iter","sim_dual_iter","sim_network_dual_deg_iter","sim_network_dual_hotstart","sim_network_dual_hotstart_lu","sim_network_dual_inf_iter","sim_network_dual_iter","sim_network_primal_deg_iter","sim_network_primal_hotstart","sim_network_primal_hotstart_lu","sim_network_primal_inf_iter","sim_network_primal_iter","sim_numcon","sim_numvar","sim_primal_deg_iter","sim_primal_dual_deg_iter","sim_primal_dual_hotstart","sim_primal_dual_hotstart_lu","sim_primal_dual_inf_iter","sim_primal_dual_iter","sim_primal_hotstart","sim_primal_hotstart_lu","sim_primal_inf_iter","sim_primal_iter","sim_solve_dual","sol_bas_prosta","sol_bas_solsta","sol_int_prosta","sol_int_solsta","sol_itg_prosta","sol_itg_solsta","sol_itr_prosta","sol_itr_solsta","sto_num_a_cache_flushes","sto_num_a_realloc","sto_num_a_transposes"], [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96])
xmlwriteroutputtype = Enum("xmlwriteroutputtype", ["col","row"], [1,0])
optimizertype = Enum("optimizertype", ["concurrent","conic","dual_simplex","free","free_simplex","intpnt","mixed_int","mixed_int_conic","network_primal_simplex","nonconvex","primal_dual_simplex","primal_simplex"], [10,2,4,0,6,1,9,8,7,11,5,3])
presolvemode = Enum("presolvemode", ["free","off","on"], [2,0,1])
miocontsoltype = Enum("miocontsoltype", ["itg","itg_rel","none","root"], [2,3,0,1])

@accepts(_accept_anyenum(rescode))
def getcodedesc(code_):
  """
  Obtains a short description of a response code.

  getcodedesc(code_)
    code: mosek.rescode. A valid response code.
  returns: symname,str
    symname: unicode. Symbolic name corresponding to the code.
    str: unicode. Obtains a short description of a response code.
  """
  symname_ = (ctypes.c_char * value.max_str_len)()
  str_ = (ctypes.c_char * value.max_str_len)()
  res = __library__.MSK_XX_getcodedesc(code_,symname_,str_)
  if res != 0:
    raise Error(rescode(res),"")
  _symname_retval = symname_.value.decode("utf-8")
  _str_retval = str_.value.decode("utf-8")
  return (_symname_retval,_str_retval)
@accepts()
def getversion():
  """
  Obtains MOSEK version information.

  getversion()
  returns: major,minor,build,revision
    major: int. Major version number.
    minor: int. Minor version number.
    build: int. Build number.
    revision: int. Revision number.
  """
  major_ = ctypes.c_int32()
  major_ = ctypes.c_int32()
  minor_ = ctypes.c_int32()
  minor_ = ctypes.c_int32()
  build_ = ctypes.c_int32()
  build_ = ctypes.c_int32()
  revision_ = ctypes.c_int32()
  revision_ = ctypes.c_int32()
  res = __library__.MSK_XX_getversion(ctypes.byref(major_),ctypes.byref(minor_),ctypes.byref(build_),ctypes.byref(revision_))
  if res != 0:
    raise Error(rescode(res),"")
  _major_return_value = major_.value
  _minor_return_value = minor_.value
  _build_return_value = build_.value
  _revision_return_value = revision_.value
  return (_major_return_value,_minor_return_value,_build_return_value,_revision_return_value)
@accepts()
def licensecleanup():
  """
  Stops all threads and delete all handles used by the license system.

  licensecleanup()
  """
  res = __library__.MSK_XX_licensecleanup()
  if res != 0:
    raise Error(rescode(res),"")

class Env:
  """
  The MOSEK environment. 
  """
  def __init__(self,licensefile=None,debugfile=None):
      self.__nativep = ctypes.c_void_p()
      self.__library = __library__
      res = self.__library.MSK_XX_makeenv(ctypes.byref(self.__nativep),debugfile)
      if res != 0:
          raise Error(rescode(res),"Error %d" % res)
      try:
          if licensefile is not None:
              res = self.__library.MSK_XX_putlicensepath(self.__nativep,licensefile)
              if res != 0:
                  raise Error(rescode(res),"Error %d" % res)
          res = self.__library.MSK_XX_initenv(self.__nativep)
          if res != 0:
              raise Error(rescode(res),"Error %d" % res)        
          
          # user stream functions: 
          self.__stream_func   = 4 * [ None ]
          # strema proxy functions and wrappers:
          self.__stream_cb   = 4 * [ None ]
          for whichstream in xrange(4): 
              # Note: Apparently closures doesn't work when the function is wrapped in a C function... So we use default parameter value instead.
              def stream_proxy(handle, msg, whichstream=whichstream):
                  func = self.__stream_func[whichstream]
                  try:
                      if func:
                          func(msg)
                  except:
                      pass
              self.__stream_cb[whichstream] = __stream_cb_type__(stream_proxy)
          self.__enablegarcolenv()
      except:
          self.__library.MSK_XX_deleteenv(ctypes.byref(self.__nativep))
          raise
      
  def set_Stream(self,whichstream,func):
      if isinstance(whichstream, streamtype):
          self.__stream_func[whichstream] = func
          if func is None:
              res = self.__library.MSK_XX_linkfunctoenvstream(self.__nativep,whichstream,None,None)
          else:
              res = self.__library.MSK_XX_linkfunctoenvstream(self.__nativep,whichstream,None,self.__stream_cb[whichstream])
      else:
          raise TypeError("Invalid stream %s" % whichstream)
  def __enablegarcolenv(self):
        self.__library.MSK_XX_enablegarcolenv(self.__nativep)

  def _getNativeP(self):
      return self.__nativep
  def __del__(self):
    if self.__nativep is not None:
      for f in feature.members():
        self.checkinlicense(f)
      self.__library.MSK_XX_deleteenv(ctypes.byref(self.__nativep))
      del self.__stream_func
      del self.__stream_cb
      del self.__library
    self.__nativep  = None
  def Task(self,maxnumcon=0,maxnumvar=0):
    return Task(self,maxnumcon,maxnumvar)

  # Implementation of disposable protocol  
  def __enter__(self):
      return self

  def __exit__(self,exc_type,exc_value,traceback):
      self.__del__()
  @accepts(_accept_any,_accept_anyenum(feature))
  def checkoutlicense(self,feature_):
    """
    Check out a license feature from the license server ahead of time.
  
    checkoutlicense(self,feature_)
      feature: mosek.feature. Feature to check out from the license system.
    """
    res = __library__.MSK_XX_checkoutlicense(self.__nativep,feature_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_accept_anyenum(feature))
  def checkinlicense(self,feature_):
    """
    Check in a license feature from the license server ahead of time.
  
    checkinlicense(self,feature_)
      feature: mosek.feature. Feature to check in to the license system.
    """
    res = __library__.MSK_XX_checkinlicense(self.__nativep,feature_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_int)
  def echointro(self,longver_):
    """
    Prints an intro to message stream.
  
    echointro(self,longver_)
      longver: int. If non-zero, then the intro is slightly longer.
    """
    res = __library__.MSK_XX_echointro(self.__nativep,longver_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_accept_anyenum(streamtype),_accept_str,_make_int)
  def linkfiletostream(self,whichstream_,filename_,append_):
    """
    Directs all output from a stream to a file.
  
    linkfiletostream(self,whichstream_,filename_,append_)
      whichstream: mosek.streamtype. Index of the stream.
      filename: str|unicode. Name of the file to write stream data to.
      append: int. If this argument is non-zero, the output is appended to the file.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_linkfiletoenvstream(self.__nativep,whichstream_,filename_,append_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_accept_str)
  def putdllpath(self,dllpath_):
    """
    Sets the path to the DLL/shared libraries that MOSEK is loading.
  
    putdllpath(self,dllpath_)
      dllpath: str|unicode. A path to the dynamic MOSEK libraries.
    """
    if isinstance(dllpath_,unicode):
      dllpath_ = dllpath_.encode("utf-8")
    res = __library__.MSK_XX_putdllpath(self.__nativep,dllpath_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_int)
  def putlicensedebug(self,licdebug_):
    """
    Enables debug information for the license system.
  
    putlicensedebug(self,licdebug_)
      licdebug: int. Enable output of license check-out debug information.
    """
    res = __library__.MSK_XX_putlicensedebug(self.__nativep,licdebug_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_intvector)
  def putlicensecode(self,code_):
    """
    The purpose of this function is to input a runtime license code.
  
    putlicensecode(self,code_)
      code: array of int. A license key string.
    """
    if code_ is not None and len(code_) < value.license_buffer_length:
      raise ValueError("Array argument code is not long enough")
    if isinstance(code_, numpy.ndarray) and code_.dtype is numpy.dtype(numpy.int32) and code_.flags.contiguous:
      _code_copyarray = False
      _code_tmp = ctypes.cast(code_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(code_,array.ndarray) and code_.dtype is numpy.dtype(array.int32) and  code_.getsteplength() == 1:
      _code_copyarray = False
      _code_tmp = code_.getdatacptr()
    elif code_ is None:
      _code_copyarray = False
      _code_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _code_copyarray = True
      _code_tmp = (ctypes.c_int32 * len(code_))(*code_)
    res = __library__.MSK_XX_putlicensecode(self.__nativep,_code_tmp)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_int)
  def putlicensewait(self,licwait_):
    """
    Control whether mosek should wait for an available license if no license is available.
  
    putlicensewait(self,licwait_)
      licwait: int. Enable waiting for a license.
    """
    res = __library__.MSK_XX_putlicensewait(self.__nativep,licwait_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_accept_str)
  def putlicensepath(self,licensepath_):
    """
    Set the path to the license file.
  
    putlicensepath(self,licensepath_)
      licensepath: str|unicode. A path specifycing where to search for the license.
    """
    if isinstance(licensepath_,unicode):
      licensepath_ = licensepath_.encode("utf-8")
    res = __library__.MSK_XX_putlicensepath(self.__nativep,licensepath_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_int)
  def putkeepdlls(self,keepdlls_):
    """
    Controls whether explicitly loaded DLLs should be kept.
  
    putkeepdlls(self,keepdlls_)
      keepdlls: int. Controls whether explicitly loaded DLLs should be kept.
    """
    res = __library__.MSK_XX_putkeepdlls(self.__nativep,keepdlls_)
    if res != 0:
      raise Error(rescode(res),"")
  @accepts(_accept_any,_make_int,_make_double,_make_doublevector,_accept_doublevector)
  def axpy(self,n_,alpha_,x_,y_):
    """
    Adds alpha times x to y.
  
    axpy(self,n_,alpha_,x_,y_)
      n: int. Length of the vectors.
      alpha: double. The scalar that multiplies x.
      x: array of double. The  vector.
      y: array of double. The  vector.
    """
    if x_ is not None and len(x_) < (n_):
      raise ValueError("Array argument x is not long enough")
    if x_ is None:
      raise ValueError("Argument x cannot be None")
    if x_ is None:
      raise ValueError("Argument x may not be None")
    if isinstance(x_, numpy.ndarray) and x_.dtype is numpy.dtype(numpy.float64) and x_.flags.contiguous:
      _x_copyarray = False
      _x_tmp = ctypes.cast(x_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(x_,array.ndarray) and x_.dtype is numpy.dtype(array.float64) and  x_.getsteplength() == 1:
      _x_copyarray = False
      _x_tmp = x_.getdatacptr()
    elif x_ is None:
      _x_copyarray = False
      _x_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _x_copyarray = True
      _x_tmp = (ctypes.c_double * len(x_))(*x_)
    if y_ is not None and len(y_) < (n_):
      raise ValueError("Array argument y is not long enough")
    if isinstance(y_,numpy.ndarray) and not y_.flags.writeable:
      raise ValueError("Argument y must be writable")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_axpy(self.__nativep,n_,alpha_,_x_tmp,_y_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _y_copyarray:
      if isinstance(y_, numpy.ndarray) and False:
        y_[:] = list(_y_tmp)
      else:
        y_[:] = _y_tmp
  @accepts(_accept_any,_make_int,_make_doublevector,_make_doublevector)
  def dot(self,n_,x_,y_):
    """
    Computes the inner product of two vectors.
  
    dot(self,n_,x_,y_)
      n: int. Length of the vectors.
      x: array of double. The x vector.
      y: array of double. The y vector.
    returns: xty
      xty: double. The result of the inner product.
    """
    if x_ is not None and len(x_) < (n_):
      raise ValueError("Array argument x is not long enough")
    if x_ is None:
      raise ValueError("Argument x cannot be None")
    if x_ is None:
      raise ValueError("Argument x may not be None")
    if isinstance(x_, numpy.ndarray) and x_.dtype is numpy.dtype(numpy.float64) and x_.flags.contiguous:
      _x_copyarray = False
      _x_tmp = ctypes.cast(x_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(x_,array.ndarray) and x_.dtype is numpy.dtype(array.float64) and  x_.getsteplength() == 1:
      _x_copyarray = False
      _x_tmp = x_.getdatacptr()
    elif x_ is None:
      _x_copyarray = False
      _x_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _x_copyarray = True
      _x_tmp = (ctypes.c_double * len(x_))(*x_)
    if y_ is not None and len(y_) < (n_):
      raise ValueError("Array argument y is not long enough")
    if y_ is None:
      raise ValueError("Argument y cannot be None")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    xty_ = ctypes.c_double()
    xty_ = ctypes.c_double()
    res = __library__.MSK_XX_dot(self.__nativep,n_,_x_tmp,_y_tmp,ctypes.byref(xty_))
    if res != 0:
      raise Error(rescode(res),"")
    _xty_return_value = xty_.value
    return (_xty_return_value)
  @accepts(_accept_any,_accept_anyenum(transpose),_make_int,_make_int,_make_double,_make_doublevector,_make_doublevector,_make_double,_accept_doublevector)
  def gemv(self,transa_,m_,n_,alpha_,a_,x_,beta_,y_):
    """
    Computes dense matrix times a dense vector product.
  
    gemv(self,transa_,m_,n_,alpha_,a_,x_,beta_,y_)
      transa: mosek.transpose. Indicates whether the matrix A must be transposed.
      m: int. Specifies the number of rows of the matrix A.
      n: int. Specifies the number of columns of the matrix A.
      alpha: double. A scalar value multipling the matrix A.
      a: array of double. A pointer to the array storing matrix A in a column-major format.
      x: array of double. A pointer to the array storing the vector x.
      beta: double. A scalar value multipling thevector y.
      y: array of double. A pointer to the array storing the vector y.
    """
    if a_ is not None and len(a_) < ((n_) * (m_)):
      raise ValueError("Array argument a is not long enough")
    if a_ is None:
      raise ValueError("Argument a cannot be None")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    if ((transa_) == transpose.no):
      __tmp_var_0 = (n_);
    else:
      __tmp_var_0 = (m_);
    if x_ is not None and len(x_) < __tmp_var_0:
      raise ValueError("Array argument x is not long enough")
    if x_ is None:
      raise ValueError("Argument x cannot be None")
    if x_ is None:
      raise ValueError("Argument x may not be None")
    if isinstance(x_, numpy.ndarray) and x_.dtype is numpy.dtype(numpy.float64) and x_.flags.contiguous:
      _x_copyarray = False
      _x_tmp = ctypes.cast(x_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(x_,array.ndarray) and x_.dtype is numpy.dtype(array.float64) and  x_.getsteplength() == 1:
      _x_copyarray = False
      _x_tmp = x_.getdatacptr()
    elif x_ is None:
      _x_copyarray = False
      _x_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _x_copyarray = True
      _x_tmp = (ctypes.c_double * len(x_))(*x_)
    if ((transa_) == transpose.no):
      __tmp_var_1 = (m_);
    else:
      __tmp_var_1 = (n_);
    if y_ is not None and len(y_) < __tmp_var_1:
      raise ValueError("Array argument y is not long enough")
    if isinstance(y_,numpy.ndarray) and not y_.flags.writeable:
      raise ValueError("Argument y must be writable")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_gemv(self.__nativep,transa_,m_,n_,alpha_,_a_tmp,_x_tmp,beta_,_y_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _y_copyarray:
      if isinstance(y_, numpy.ndarray) and False:
        y_[:] = list(_y_tmp)
      else:
        y_[:] = _y_tmp
  @accepts(_accept_any,_accept_anyenum(transpose),_accept_anyenum(transpose),_make_int,_make_int,_make_int,_make_double,_make_doublevector,_make_doublevector,_make_double,_accept_doublevector)
  def gemm(self,transa_,transb_,m_,n_,k_,alpha_,a_,b_,beta_,c_):
    """
    Performs a dense matrix multiplication.
  
    gemm(self,transa_,transb_,m_,n_,k_,alpha_,a_,b_,beta_,c_)
      transa: mosek.transpose. Indicates whether the matrix A must be transposed.
      transb: mosek.transpose. Indicates whether the matrix B must be transposed.
      m: int. Indicates the number of rows of matrices A and C.
      n: int. Indicates the number of columns of matrices B and C.
      k: int. Specifies the number of columns of the matrix A and the number of rows of the matrix B.
      alpha: double. A scalar value multipling the result of the matrix multiplication.
      a: array of double. The pointer to the array storing matrix A in a column-major format.
      b: array of double. Indicates the number of rows of matrix B and columns of matrix A.
      beta: double. A scalar value that multiplies C.
      c: array of double. The pointer to the array storing matrix C in a column-major format.
    """
    if a_ is not None and len(a_) < ((m_) * (k_)):
      raise ValueError("Array argument a is not long enough")
    if a_ is None:
      raise ValueError("Argument a cannot be None")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    if b_ is not None and len(b_) < ((k_) * (n_)):
      raise ValueError("Array argument b is not long enough")
    if b_ is None:
      raise ValueError("Argument b cannot be None")
    if b_ is None:
      raise ValueError("Argument b may not be None")
    if isinstance(b_, numpy.ndarray) and b_.dtype is numpy.dtype(numpy.float64) and b_.flags.contiguous:
      _b_copyarray = False
      _b_tmp = ctypes.cast(b_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(b_,array.ndarray) and b_.dtype is numpy.dtype(array.float64) and  b_.getsteplength() == 1:
      _b_copyarray = False
      _b_tmp = b_.getdatacptr()
    elif b_ is None:
      _b_copyarray = False
      _b_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _b_copyarray = True
      _b_tmp = (ctypes.c_double * len(b_))(*b_)
    if c_ is not None and len(c_) < ((m_) * (n_)):
      raise ValueError("Array argument c is not long enough")
    if isinstance(c_,numpy.ndarray) and not c_.flags.writeable:
      raise ValueError("Argument c must be writable")
    if c_ is None:
      raise ValueError("Argument c may not be None")
    if isinstance(c_, numpy.ndarray) and c_.dtype is numpy.dtype(numpy.float64) and c_.flags.contiguous:
      _c_copyarray = False
      _c_tmp = ctypes.cast(c_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(c_,array.ndarray) and c_.dtype is numpy.dtype(array.float64) and  c_.getsteplength() == 1:
      _c_copyarray = False
      _c_tmp = c_.getdatacptr()
    elif c_ is None:
      _c_copyarray = False
      _c_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _c_copyarray = True
      _c_tmp = (ctypes.c_double * len(c_))(*c_)
    res = __library__.MSK_XX_gemm(self.__nativep,transa_,transb_,m_,n_,k_,alpha_,_a_tmp,_b_tmp,beta_,_c_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _c_copyarray:
      if isinstance(c_, numpy.ndarray) and False:
        c_[:] = list(_c_tmp)
      else:
        c_[:] = _c_tmp
  @accepts(_accept_any,_accept_anyenum(uplo),_accept_anyenum(transpose),_make_int,_make_int,_make_double,_make_doublevector,_make_double,_accept_doublevector)
  def syrk(self,uplo_,trans_,n_,k_,alpha_,a_,beta_,c_):
    """
    Performs a rank-k update of a symmetric matrix.
  
    syrk(self,uplo_,trans_,n_,k_,alpha_,a_,beta_,c_)
      uplo: mosek.uplo. Indicates whether the upper or lower triangular part of C is stored.
      trans: mosek.transpose. Indicates whether the matrix A must be transposed.
      n: int. Specifies the order of C.
      k: int. Indicates the number of rows or columns of A, and its rank.
      alpha: double. A scalar value multipling the result of the matrix multiplication.
      a: array of double. The pointer to the array storing matrix A in a column-major format.
      beta: double. A scalar value that multiplies C.
      c: array of double. The pointer to the array storing matrix C in a column-major format.
    """
    if a_ is not None and len(a_) < ((n_) * (k_)):
      raise ValueError("Array argument a is not long enough")
    if a_ is None:
      raise ValueError("Argument a cannot be None")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    if c_ is not None and len(c_) < ((n_) * (n_)):
      raise ValueError("Array argument c is not long enough")
    if isinstance(c_,numpy.ndarray) and not c_.flags.writeable:
      raise ValueError("Argument c must be writable")
    if c_ is None:
      raise ValueError("Argument c may not be None")
    if isinstance(c_, numpy.ndarray) and c_.dtype is numpy.dtype(numpy.float64) and c_.flags.contiguous:
      _c_copyarray = False
      _c_tmp = ctypes.cast(c_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(c_,array.ndarray) and c_.dtype is numpy.dtype(array.float64) and  c_.getsteplength() == 1:
      _c_copyarray = False
      _c_tmp = c_.getdatacptr()
    elif c_ is None:
      _c_copyarray = False
      _c_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _c_copyarray = True
      _c_tmp = (ctypes.c_double * len(c_))(*c_)
    res = __library__.MSK_XX_syrk(self.__nativep,uplo_,trans_,n_,k_,alpha_,_a_tmp,beta_,_c_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _c_copyarray:
      if isinstance(c_, numpy.ndarray) and False:
        c_[:] = list(_c_tmp)
      else:
        c_[:] = _c_tmp
  @accepts(_accept_any,_accept_anyenum(uplo),_make_int,_accept_doublevector)
  def potrf(self,uplo_,n_,a_):
    """
    Computes a Cholesky factorization a dense matrix.
  
    potrf(self,uplo_,n_,a_)
      uplo: mosek.uplo. Indicates whether the upper or lower triangular part of the matrix is stored.
      n: int. Dimension of the symmetric matrix.
      a: array of double. A symmetric matrix stored in column-major order. Only the lower or the upper triangular part is used, accordingly with the uplo parameter. It will contain the result on exit.
    """
    if a_ is not None and len(a_) < ((n_) * (n_)):
      raise ValueError("Array argument a is not long enough")
    if isinstance(a_,numpy.ndarray) and not a_.flags.writeable:
      raise ValueError("Argument a must be writable")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    res = __library__.MSK_XX_potrf(self.__nativep,uplo_,n_,_a_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _a_copyarray:
      if isinstance(a_, numpy.ndarray) and False:
        a_[:] = list(_a_tmp)
      else:
        a_[:] = _a_tmp
  @accepts(_accept_any,_accept_anyenum(uplo),_make_int,_make_doublevector,_accept_doublevector)
  def syeig(self,uplo_,n_,a_,w_):
    """
    Computes all eigenvalues of a symmetric dense matrix.
  
    syeig(self,uplo_,n_,a_,w_)
      uplo: mosek.uplo. Indicates whether the upper or lower triangular part is used.
      n: int. Dimension of the symmetric input matrix.
      a: array of double. A symmetric matrix stored in column-major order. Only the lower-triangular part is used.
      w: array of double. Array of minimum dimension n where eigenvalues will be stored.
    """
    if a_ is not None and len(a_) < ((n_) * (n_)):
      raise ValueError("Array argument a is not long enough")
    if a_ is None:
      raise ValueError("Argument a cannot be None")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    if w_ is not None and len(w_) < (n_):
      raise ValueError("Array argument w is not long enough")
    if isinstance(w_,numpy.ndarray) and not w_.flags.writeable:
      raise ValueError("Argument w must be writable")
    if w_ is None:
      raise ValueError("Argument w may not be None")
    if isinstance(w_, numpy.ndarray) and w_.dtype is numpy.dtype(numpy.float64) and w_.flags.contiguous:
      _w_copyarray = False
      _w_tmp = ctypes.cast(w_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(w_,array.ndarray) and w_.dtype is numpy.dtype(array.float64) and  w_.getsteplength() == 1:
      _w_copyarray = False
      _w_tmp = w_.getdatacptr()
    elif w_ is None:
      _w_copyarray = False
      _w_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _w_copyarray = True
      _w_tmp = (ctypes.c_double * len(w_))(*w_)
    res = __library__.MSK_XX_syeig(self.__nativep,uplo_,n_,_a_tmp,_w_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _w_copyarray:
      if isinstance(w_, numpy.ndarray) and False:
        w_[:] = list(_w_tmp)
      else:
        w_[:] = _w_tmp
  @accepts(_accept_any,_accept_anyenum(uplo),_make_int,_accept_doublevector,_accept_doublevector)
  def syevd(self,uplo_,n_,a_,w_):
    """
    Computes all the eigenvalue and eigenvectors of a symmetric dense matrix, and thus its eigenvalue decomposition.
  
    syevd(self,uplo_,n_,a_,w_)
      uplo: mosek.uplo. Indicates whether the upper or lower triangular part is used.
      n: int. Dimension of symmetric input matrix.
      a: array of double. A symmetric matrix stored in column-major order. Only the lower-triangular part is used. It will be overwritten on exit.
      w: array of double. An array where eigenvalues will be stored. Its lenght must be at least the dimension of the input matrix.
    """
    if a_ is not None and len(a_) < ((n_) * (n_)):
      raise ValueError("Array argument a is not long enough")
    if isinstance(a_,numpy.ndarray) and not a_.flags.writeable:
      raise ValueError("Argument a must be writable")
    if a_ is None:
      raise ValueError("Argument a may not be None")
    if isinstance(a_, numpy.ndarray) and a_.dtype is numpy.dtype(numpy.float64) and a_.flags.contiguous:
      _a_copyarray = False
      _a_tmp = ctypes.cast(a_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(a_,array.ndarray) and a_.dtype is numpy.dtype(array.float64) and  a_.getsteplength() == 1:
      _a_copyarray = False
      _a_tmp = a_.getdatacptr()
    elif a_ is None:
      _a_copyarray = False
      _a_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _a_copyarray = True
      _a_tmp = (ctypes.c_double * len(a_))(*a_)
    if w_ is not None and len(w_) < (n_):
      raise ValueError("Array argument w is not long enough")
    if isinstance(w_,numpy.ndarray) and not w_.flags.writeable:
      raise ValueError("Argument w must be writable")
    if w_ is None:
      raise ValueError("Argument w may not be None")
    if isinstance(w_, numpy.ndarray) and w_.dtype is numpy.dtype(numpy.float64) and w_.flags.contiguous:
      _w_copyarray = False
      _w_tmp = ctypes.cast(w_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(w_,array.ndarray) and w_.dtype is numpy.dtype(array.float64) and  w_.getsteplength() == 1:
      _w_copyarray = False
      _w_tmp = w_.getdatacptr()
    elif w_ is None:
      _w_copyarray = False
      _w_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _w_copyarray = True
      _w_tmp = (ctypes.c_double * len(w_))(*w_)
    res = __library__.MSK_XX_syevd(self.__nativep,uplo_,n_,_a_tmp,_w_tmp)
    if res != 0:
      raise Error(rescode(res),"")
    if _a_copyarray:
      if isinstance(a_, numpy.ndarray) and False:
        a_[:] = list(_a_tmp)
      else:
        a_[:] = _a_tmp
    if _w_copyarray:
      if isinstance(w_, numpy.ndarray) and False:
        w_[:] = list(_w_tmp)
      else:
        w_[:] = _w_tmp

class Task:
  """
  The MOSEK task class. This object contains information about one optimization problem.
  """
  
  def __init__(self,env=None,maxnumcon=0,maxnumvar=0,nativep=None,other=None):
      """
      Construct a new Task object.
      
      Task(env=None,maxnumcon=0,maxnumvar=0,nativep=None,other=None)
        env: mosek.Env. 
        maxnumcon: int. Reserve space for this number of constraints. Default is 0. 
        maxnumcvar: int. Reserve space for this number of variables. Default is 0. 
        nativep: native pointer. For internal use only.
        other: mosek.Task. Another task.
      
      Valid usage:
        Specifying "env", and optionally "maxnumcon" and "maxnumvar" will create a new Task.
        Specifying "nativep" will create a new Task from the native mosek task defined by the pointer.
        Specifying "other" will create a new Task as a copy of the other task. 
      """
      self.__library = __library__
      self.__nativep = None
      self.__schandle = None

      if isinstance(env,Task):
          other = env
          env = None
      
      try: 
          if nativep is not None:
              self.__nativep = nativep
              res = 0
          elif other is not None:
              self.__nativep = ctypes.c_void_p()
              res = self.__library.MSK_XX_clonetask(other.__nativep, ctypes.byref(self.__nativep))
          else:
              if not isinstance(env,Env):
                  raise TypeError('Expected an Env for argument')
              self.__nativep = ctypes.c_void_p()
              res = self.__library.MSK_XX_maketask(env._getNativeP(),maxnumcon,maxnumvar,ctypes.byref(self.__nativep))
          if res != 0:
              raise Error(rescode(res),"Error %d" % res)

          # user progress function:
          self.__progress_func = None
          # callback proxy function definition:
          def progress_proxy(nativep, handle, caller, dinfptr, iinfptr, liinfptr):
              r = 0
              try:
                  if self.__progress_func:
                      caller = callbackcode(caller)
                      f = self.__progress_func
                      if f is not None:
                          r = f(caller,
                                dinfptr [:len(dinfitem._values)],
                                iinfptr [:len(iinfitem._values)],
                                liinfptr[:len(liinfitem._values)])
                      if not isinstance(r,int):
                          r = 0
              except:
                  import traceback
                  traceback.print_exc()
                  return -1
              return r
          # callback proxy C wrapper:
          self.__progress_cb = __progress_cb_type__(progress_proxy)
        
          # user stream functions: 
          self.__stream_func   = 4 * [ None ]
          # strema proxy functions and wrappers:
          self.__stream_cb   = 4 * [ None ]
          for whichstream in xrange(4): 
              # Note: Apparently closures doesn't work when the function is wrapped in a C function... So we use default parameter value instead.
              def stream_proxy(handle, msg, whichstream=whichstream):
                  func = self.__stream_func[whichstream]
                  try:
                      if func is not None:
                          func(msg)
                  except:
                      pass
              self.__stream_cb[whichstream] = __stream_cb_type__(stream_proxy)
          assert self.__nativep



      except:
          #import traceback
          #traceback.print_exc()
          if hasattr(self,'_Task__nativep') and self.__nativep is not None:
              #print "DELETE TASK 2",id(self)
              self.__library.MSK_XX_deletetask(ctypes.byref(self.__nativep))
              self.__nativep = None
          raise
      
  def __del__(self):
      #print "DELETE TASK 1",id(self)
      if self.__nativep is not None:
          self.removeSCeval()        
          self.__library.MSK_XX_deletetask(ctypes.byref(self.__nativep))
          del self.__library
          del self.__schandle
          del self.__progress_func
          del self.__progress_cb
          del self.__stream_func
          del self.__stream_cb
      self.__nativep = None

  def __enter__(self):
      return self

  def __exit__(self,exc_type,exc_value,traceback):
      self.__del__()
      

  def __getlasterror(self,res):
      msglen = ctypes.c_size_t(1024)
      lasterr = ctypes.c_int()
      r = self.__library.MSK_XX_getlasterror64(self.__nativep, ctypes.byref(lasterr), 0, ctypes.byref(msglen),None)
      if r == 0:
          #msg = (ctypes.c_char * (msglen.value+1))()
          len = (msglen.value+1)
          msg = ctypes.create_string_buffer(len)
          r = self.__library.MSK_XX_getlasterror64(self.__nativep, ctypes.byref(lasterr), len, None,msg)
          if r == 0:
              result,msg = lasterr.value,msg.value.decode('utf-8')
          else:
              result,msg = lasterr.value,''
      else:
          result,msg = res,''
      return result,msg


  def set_Progress(self,func):
      """
      Set the progress callback function. If func is None, progress callbacks are detached and disabled.
      """
      if func is None:
          self.__progress_func = None
          res = self.__library.MSK_XX_putcallbackfunc(self.__nativep,None,None)
      else:
          self.__progress_func = func          
          res = self.__library.MSK_XX_putcallbackfunc(self.__nativep,self.__progress_cb,None)

  def set_Stream(self,whichstream,func):
      if isinstance(whichstream, streamtype):
          if func is None:
              self.__stream_func[whichstream] = None
              res = self.__library.MSK_XX_linkfunctotaskstream(self.__nativep,whichstream,None,None)
          else:
              self.__stream_func[whichstream] = func
              res = self.__library.MSK_XX_linkfunctotaskstream(self.__nativep,whichstream,None,self.__stream_cb[whichstream])
      else:
          raise TypeError("Invalid stream %s" % whichstream)  


  ## SCopt functionality must currently be hand-written since it is not well-integrated with Mosek.

  def removeSCeval(self):
    if self.__schandle is not None:
        __load_scopt__()
        __scopt__.MSK_scend(self.__nativep, self.__schandle)
        self.__schandle = None
    
  def putSCeval(self,
                opro  = None, 
                oprjo = None,
                oprfo = None,
                oprgo = None,
                oprho = None,
                oprc  = None,
                opric = None,
                oprjc = None,
                oprfc = None,
                oprgc = None,
                oprhc = None):
    """
    Input data for SCopt. If other SCopt data was inputted before, the new data replaces the old.
    
    Defining a non-liner objective requires that all of the arguments opro,
    oprjo, oprfo, oprgo and oprho are defined. If present, all these arrays
    must have the same length.
    Defining non-linear constraints requires that all of the arguments oprc,
    opric, oprjc, oprfc, oprgc and oprhc are defined. If present, all these
    arrays must have the same length.

    Keyword arguments:
      [opro]  Array of mosek.scopr values. Defines the functions used for the objective.
      [oprjo] Array of indexes. Defines the variable indexes used in non-linear objective function.
      [oprfo] Array of coefficients. Defines constants used in the objective.
      [oprgo] Array of coefficients. Defines constants used in the objective.
      [oprho] Array of coefficients. Defines constants used in the objective.
      [oprc]  Array of mosek.scopr values. Defines the functions used for the constraints.
      [opric] Array of indexes. Defines the variable indexes used in the non-linear constraint functions.
      [oprjc] Array of indexes. Defines the constraint indexes where non-linear functions appear.
      [oprfc] Array of coefficients. Defines constants used in the non-linear constraints.
      [oprgc] Array of coefficients. Defines constants used in the non-linear constraints.
      [oprhc] Array of coefficients. Defines constants used in the non-linear constraints.
    """
    
    __load_scopt__()
    if self.__schandle is not None:
        __scopt__.MSK_scend(self.__nativep, self.__schandle)
        self.__schandle = None
    numvar = self.getnumvar()
    numcon = self.getnumcon()

    _opro = None
    _oprc = None

    if (    opro  is not None 
        and oprjo is not None 
        and oprfo is not None 
        and oprgo is not None
        and oprho is not None):
        try:
            numnlov = len(opro)
            if (   numnlov != len(oprjo)
                or numnlov != len(oprfo)
                or numnlov != len(oprgo)
                or numnlov != len(oprho)):
                raise ValueError("Arguments opro, oprjo, oprfo, oprgo and oprho have different lengths") 
            for i in opro:
                if not isinstance(i,scopr):
                    raise SCoptException("Argument opro must be an array of mosek.scopr")
            _opro  = array.array(opro, array.int32)
            _oprjo = array.array(oprjo,array.int32)
            _oprfo = array.array(oprfo,float)
            _oprgo = array.array(oprgo,float)
            _oprho = array.array(oprho,float)

            for j in _oprjo: 
                if j < 0 or j >= numvar: raise SCoptException("Invalid variable index oprjo entry");

        except TypeError:
            raise ValueError("Arguments opro, oprjo, oprfo, oprgo and oprho must be arrays") 
    else:
        numnlov = 0

    if (    oprc  is not None 
        and opric is not None 
        and oprjc is not None 
        and oprfc is not None 
        and oprgc is not None
        and oprhc is not None):
        try:
            numnlcv = len(oprc)
            if (   numnlcv != len(opric)
                or numnlcv != len(oprjc)
                or numnlcv != len(oprfc)
                or numnlcv != len(oprgc)
                or numnlcv != len(oprhc)):
                raise ValueError("Arguments oprc, opric, oprjc, oprfc, oprgc and oprhc have different lengths") 
            for i in oprc:
                if not isinstance(i,scopr):
                    raise SCoptException("Argument oprc must be an array of mosek.scopr")
            _oprc  = array.array(oprc, array.int32)
            _opric = array.array(opric,array.int32)
            _oprjc = array.array(oprjc,array.int32)
            _oprfc = array.array(oprfc,float)
            _oprgc = array.array(oprgc,float)
            _oprhc = array.array(oprhc,float)
            
            for i in _opric: 
                if i < 0 or i >= numcon: raise SCoptException("Invalid variable index opric entry");
            for j in _oprjc: 
                if j < 0 or j >= numvar: raise SCoptException("Invalid variable index oprjc entry");
        except TypeError:
            # not 'len' operation
            raise ValueError("Arguments oprc, opric, oprjc, oprfc, oprgc and oprhc must be arrays") 
    else:
        numnlcv = 0
    
    def _convint32arr(arg):
        if arg is None:
          return None
        else:
          return (ctypes.c_int32 * len(arg))(*arg)
    def _convdoublearr(arg):
        if arg is None:
          return None
        else:
          return (ctypes.c_double * len(arg))(*arg)
   
    if numnlov > 0 or numnlcv > 0:
        args = [ self.__nativep ] 
        args.append(numnlov)
        if numnlov > 0:
            args.append(_convint32arr(_opro))
            args.append(_convint32arr(_oprjo))
            args.append(_convdoublearr(_oprfo))
            args.append(_convdoublearr(_oprgo))
            args.append(_convdoublearr(_oprho))
        else:
            args.extend([ None, None, None, None, None ])
            
        args.append(numnlcv)
        if numnlcv > 0:
            args.append(_convint32arr(_oprc))
            args.append(_convint32arr(_opric))
            args.append(_convint32arr(_oprjc))
            args.append(_convdoublearr(_oprfc))
            args.append(_convdoublearr(_oprgc))
            args.append(_convdoublearr(_oprhc))
        else:
            args.extend([ None, None, None, None, None, None ])

        self.__schandle = ctypes.c_void_p()
        args.append(ctypes.byref(self.__schandle))
        
        res = __scopt__.MSK_scbegin(*args)
        if res != 0:
            result,msg = self.__getlasterror(res)
            raise Error(rescode(result),msg)
        else:
            __scopt__.MSK_scattach(self.__nativep,self.__schandle)

  @accepts(_accept_any,_accept_anyenum(streamtype))
  def analyzeproblem(self,whichstream_):
    """
    Analyze the data of a task.
  
    analyzeproblem(self,whichstream_)
      whichstream: mosek.streamtype. Index of the stream.
    """
    res = __library__.MSK_XX_analyzeproblem(self.__nativep,whichstream_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype),_accept_anyenum(nametype))
  def analyzenames(self,whichstream_,nametype_):
    """
    Analyze the names and issue an error for the first invalid name.
  
    analyzenames(self,whichstream_,nametype_)
      whichstream: mosek.streamtype. Index of the stream.
      nametype: mosek.nametype. The type of names e.g. valid in MPS or LP files.
    """
    res = __library__.MSK_XX_analyzenames(self.__nativep,whichstream_,nametype_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype),_accept_anyenum(soltype))
  def analyzesolution(self,whichstream_,whichsol_):
    """
    Print information related to the quality of the solution.
  
    analyzesolution(self,whichstream_,whichsol_)
      whichstream: mosek.streamtype. Index of the stream.
      whichsol: mosek.soltype. Selects a solution.
    """
    res = __library__.MSK_XX_analyzesolution(self.__nativep,whichstream_,whichsol_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_intvector)
  def initbasissolve(self,basis_):
    """
    Prepare a task for basis solver.
  
    initbasissolve(self,basis_)
      basis: array of int. The array of basis indexes to use.
    """
    if basis_ is not None and len(basis_) < self.getnumcon():
      raise ValueError("Array argument basis is not long enough")
    if isinstance(basis_,numpy.ndarray) and not basis_.flags.writeable:
      raise ValueError("Argument basis must be writable")
    if isinstance(basis_, numpy.ndarray) and basis_.dtype is numpy.dtype(numpy.int32) and basis_.flags.contiguous:
      _basis_copyarray = False
      _basis_tmp = ctypes.cast(basis_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(basis_,array.ndarray) and basis_.dtype is numpy.dtype(array.int32) and  basis_.getsteplength() == 1:
      _basis_copyarray = False
      _basis_tmp = basis_.getdatacptr()
    elif basis_ is None:
      _basis_copyarray = False
      _basis_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _basis_copyarray = True
      _basis_tmp = (ctypes.c_int32 * len(basis_))(*basis_)
    res = __library__.MSK_XX_initbasissolve(self.__nativep,_basis_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _basis_copyarray:
      if isinstance(basis_, numpy.ndarray) and False:
        basis_[:] = list(_basis_tmp)
      else:
        basis_[:] = _basis_tmp
  @accepts(_accept_any,_make_int,_make_int,_accept_intvector,_accept_doublevector)
  def solvewithbasis(self,transp_,numnz_,sub_,val_):
    """
    Solve a linear equation system involving a basis matrix.
  
    solvewithbasis(self,transp_,numnz_,sub_,val_)
      transp: int. Controls which problem formulation is solved.
      numnz: int. Input (number of non-zeros in right-hand side) and output (number of non-zeros in solution vector).
      sub: array of int. Input (indexes of non-zeros in right-hand side) and output (indexes of non-zeros in solution vector).
      val: array of double. Input (right-hand side values) and output (solution vector values).
    returns: numnz
      numnz: int. Input (number of non-zeros in right-hand side) and output (number of non-zeros in solution vector).
    """
    _numnz_tmp = ctypes.c_int32(numnz_)
    if sub_ is not None and len(sub_) < self.getnumcon():
      raise ValueError("Array argument sub is not long enough")
    if isinstance(sub_,numpy.ndarray) and not sub_.flags.writeable:
      raise ValueError("Argument sub must be writable")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if val_ is not None and len(val_) < self.getnumcon():
      raise ValueError("Array argument val is not long enough")
    if isinstance(val_,numpy.ndarray) and not val_.flags.writeable:
      raise ValueError("Argument val must be writable")
    if isinstance(val_, numpy.ndarray) and val_.dtype is numpy.dtype(numpy.float64) and val_.flags.contiguous:
      _val_copyarray = False
      _val_tmp = ctypes.cast(val_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(val_,array.ndarray) and val_.dtype is numpy.dtype(array.float64) and  val_.getsteplength() == 1:
      _val_copyarray = False
      _val_tmp = val_.getdatacptr()
    elif val_ is None:
      _val_copyarray = False
      _val_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _val_copyarray = True
      _val_tmp = (ctypes.c_double * len(val_))(*val_)
    res = __library__.MSK_XX_solvewithbasis(self.__nativep,transp_,ctypes.byref(_numnz_tmp),_sub_tmp,_val_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numnz_return_value = _numnz_tmp.value
    if _sub_copyarray:
      if isinstance(sub_, numpy.ndarray) and False:
        sub_[:] = list(_sub_tmp)
      else:
        sub_[:] = _sub_tmp
    if _val_copyarray:
      if isinstance(val_, numpy.ndarray) and False:
        val_[:] = list(_val_tmp)
      else:
        val_[:] = _val_tmp
    return (_numnz_return_value)
  @accepts(_accept_any)
  def basiscond(self):
    """
    Computes conditioning information for the basis matrix.
  
    basiscond(self)
    returns: nrmbasis,nrminvbasis
      nrmbasis: double. An estimate for the 1 norm of the basis.
      nrminvbasis: double. An estimate for the 1 norm of the inverse of the basis.
    """
    nrmbasis_ = ctypes.c_double()
    nrmbasis_ = ctypes.c_double()
    nrminvbasis_ = ctypes.c_double()
    nrminvbasis_ = ctypes.c_double()
    res = __library__.MSK_XX_basiscond(self.__nativep,ctypes.byref(nrmbasis_),ctypes.byref(nrminvbasis_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nrmbasis_return_value = nrmbasis_.value
    _nrminvbasis_return_value = nrminvbasis_.value
    return (_nrmbasis_return_value,_nrminvbasis_return_value)
  @accepts(_accept_any,_make_int)
  def appendcons(self,num_):
    """
    Appends a number of constraints to the optimization task.
  
    appendcons(self,num_)
      num: int. Number of constraints which should be appended.
    """
    res = __library__.MSK_XX_appendcons(self.__nativep,num_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def appendvars(self,num_):
    """
    Appends a number of variables to the optimization task.
  
    appendvars(self,num_)
      num: int. Number of variables which should be appended.
    """
    res = __library__.MSK_XX_appendvars(self.__nativep,num_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector)
  def removecons(self,subset_):
    """
    The function removes a number of constraints.
  
    removecons(self,subset_)
      subset: array of int. Indexes of constraints which should be removed.
    """
    num_ = None
    if num_ is None:
      num_ = len(subset_)
    else:
      num_ = min(num_,len(subset_))
    if subset_ is None:
      raise ValueError("Argument subset cannot be None")
    if subset_ is None:
      raise ValueError("Argument subset may not be None")
    if isinstance(subset_, numpy.ndarray) and subset_.dtype is numpy.dtype(numpy.int32) and subset_.flags.contiguous:
      _subset_copyarray = False
      _subset_tmp = ctypes.cast(subset_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subset_,array.ndarray) and subset_.dtype is numpy.dtype(array.int32) and  subset_.getsteplength() == 1:
      _subset_copyarray = False
      _subset_tmp = subset_.getdatacptr()
    elif subset_ is None:
      _subset_copyarray = False
      _subset_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subset_copyarray = True
      _subset_tmp = (ctypes.c_int32 * len(subset_))(*subset_)
    res = __library__.MSK_XX_removecons(self.__nativep,num_,_subset_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector)
  def removevars(self,subset_):
    """
    The function removes a number of variables.
  
    removevars(self,subset_)
      subset: array of int. Indexes of variables which should be removed.
    """
    num_ = None
    if num_ is None:
      num_ = len(subset_)
    else:
      num_ = min(num_,len(subset_))
    if subset_ is None:
      raise ValueError("Argument subset cannot be None")
    if subset_ is None:
      raise ValueError("Argument subset may not be None")
    if isinstance(subset_, numpy.ndarray) and subset_.dtype is numpy.dtype(numpy.int32) and subset_.flags.contiguous:
      _subset_copyarray = False
      _subset_tmp = ctypes.cast(subset_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subset_,array.ndarray) and subset_.dtype is numpy.dtype(array.int32) and  subset_.getsteplength() == 1:
      _subset_copyarray = False
      _subset_tmp = subset_.getdatacptr()
    elif subset_ is None:
      _subset_copyarray = False
      _subset_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subset_copyarray = True
      _subset_tmp = (ctypes.c_int32 * len(subset_))(*subset_)
    res = __library__.MSK_XX_removevars(self.__nativep,num_,_subset_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector)
  def removebarvars(self,subset_):
    """
    The function removes a number of symmetric matrix.
  
    removebarvars(self,subset_)
      subset: array of int. Indexes of symmetric matrix which should be removed.
    """
    num_ = None
    if num_ is None:
      num_ = len(subset_)
    else:
      num_ = min(num_,len(subset_))
    if subset_ is None:
      raise ValueError("Argument subset cannot be None")
    if subset_ is None:
      raise ValueError("Argument subset may not be None")
    if isinstance(subset_, numpy.ndarray) and subset_.dtype is numpy.dtype(numpy.int32) and subset_.flags.contiguous:
      _subset_copyarray = False
      _subset_tmp = ctypes.cast(subset_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subset_,array.ndarray) and subset_.dtype is numpy.dtype(array.int32) and  subset_.getsteplength() == 1:
      _subset_copyarray = False
      _subset_tmp = subset_.getdatacptr()
    elif subset_ is None:
      _subset_copyarray = False
      _subset_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subset_copyarray = True
      _subset_tmp = (ctypes.c_int32 * len(subset_))(*subset_)
    res = __library__.MSK_XX_removebarvars(self.__nativep,num_,_subset_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector)
  def removecones(self,subset_):
    """
    Removes a conic constraint from the problem.
  
    removecones(self,subset_)
      subset: array of int. Indexes of cones which should be removed.
    """
    num_ = None
    if num_ is None:
      num_ = len(subset_)
    else:
      num_ = min(num_,len(subset_))
    if subset_ is None:
      raise ValueError("Argument subset cannot be None")
    if subset_ is None:
      raise ValueError("Argument subset may not be None")
    if isinstance(subset_, numpy.ndarray) and subset_.dtype is numpy.dtype(numpy.int32) and subset_.flags.contiguous:
      _subset_copyarray = False
      _subset_tmp = ctypes.cast(subset_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subset_,array.ndarray) and subset_.dtype is numpy.dtype(array.int32) and  subset_.getsteplength() == 1:
      _subset_copyarray = False
      _subset_tmp = subset_.getdatacptr()
    elif subset_ is None:
      _subset_copyarray = False
      _subset_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subset_copyarray = True
      _subset_tmp = (ctypes.c_int32 * len(subset_))(*subset_)
    res = __library__.MSK_XX_removecones(self.__nativep,num_,_subset_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector)
  def appendbarvars(self,dim_):
    """
    Appends a semidefinite  variable of dimension dim to the problem.
  
    appendbarvars(self,dim_)
      dim: array of int. Dimension of symmetric matrix variables to be added.
    """
    num_ = None
    if num_ is None:
      num_ = len(dim_)
    else:
      num_ = min(num_,len(dim_))
    if dim_ is None:
      raise ValueError("Argument dim cannot be None")
    if dim_ is None:
      raise ValueError("Argument dim may not be None")
    if isinstance(dim_, numpy.ndarray) and dim_.dtype is numpy.dtype(numpy.int32) and dim_.flags.contiguous:
      _dim_copyarray = False
      _dim_tmp = ctypes.cast(dim_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(dim_,array.ndarray) and dim_.dtype is numpy.dtype(array.int32) and  dim_.getsteplength() == 1:
      _dim_copyarray = False
      _dim_tmp = dim_.getdatacptr()
    elif dim_ is None:
      _dim_copyarray = False
      _dim_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _dim_copyarray = True
      _dim_tmp = (ctypes.c_int32 * len(dim_))(*dim_)
    res = __library__.MSK_XX_appendbarvars(self.__nativep,num_,_dim_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(conetype),_make_double,_make_intvector)
  def appendcone(self,conetype_,conepar_,submem_):
    """
    Appends a new cone constraint to the problem.
  
    appendcone(self,conetype_,conepar_,submem_)
      conetype: mosek.conetype. Specifies the type of the cone.
      conepar: double. This argument is currently not used. Can be set to 0.0.
      submem: array of int. Variable subscripts of the members in the cone.
    """
    nummem_ = None
    if nummem_ is None:
      nummem_ = len(submem_)
    else:
      nummem_ = min(nummem_,len(submem_))
    if submem_ is None:
      raise ValueError("Argument submem cannot be None")
    if submem_ is None:
      raise ValueError("Argument submem may not be None")
    if isinstance(submem_, numpy.ndarray) and submem_.dtype is numpy.dtype(numpy.int32) and submem_.flags.contiguous:
      _submem_copyarray = False
      _submem_tmp = ctypes.cast(submem_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(submem_,array.ndarray) and submem_.dtype is numpy.dtype(array.int32) and  submem_.getsteplength() == 1:
      _submem_copyarray = False
      _submem_tmp = submem_.getdatacptr()
    elif submem_ is None:
      _submem_copyarray = False
      _submem_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _submem_copyarray = True
      _submem_tmp = (ctypes.c_int32 * len(submem_))(*submem_)
    res = __library__.MSK_XX_appendcone(self.__nativep,conetype_,conepar_,nummem_,_submem_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(conetype),_make_double,_make_int,_make_int)
  def appendconeseq(self,conetype_,conepar_,nummem_,j_):
    """
    Appends a new conic constraint to the problem.
  
    appendconeseq(self,conetype_,conepar_,nummem_,j_)
      conetype: mosek.conetype. Specifies the type of the cone.
      conepar: double. This argument is currently not used. Can be set to 0.0.
      nummem: int. Dimension of the conic constraint.
      j: int. Index of the first variable in the conic constraint.
    """
    res = __library__.MSK_XX_appendconeseq(self.__nativep,conetype_,conepar_,nummem_,j_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_anyenumvector(conetype),_make_doublevector,_make_intvector,_make_int)
  def appendconesseq(self,conetype_,conepar_,nummem_,j_):
    """
    Appends multiple conic constraints to the problem.
  
    appendconesseq(self,conetype_,conepar_,nummem_,j_)
      conetype: array of mosek.conetype. Specifies the type of the cone.
      conepar: array of double. This argument is currently not used. Can be set to 0.0.
      nummem: array of int. Number of member variables in the cone.
      j: int. Index of the first variable in the first cone to be appended.
    """
    num_ = None
    if num_ is None:
      num_ = len(conetype_)
    else:
      num_ = min(num_,len(conetype_))
    if num_ is None:
      num_ = len(conepar_)
    else:
      num_ = min(num_,len(conepar_))
    if num_ is None:
      num_ = len(nummem_)
    else:
      num_ = min(num_,len(nummem_))
    if conetype_ is None:
      raise ValueError("Argument conetype cannot be None")
    if conetype_ is None:
      raise ValueError("Argument conetype may not be None")
    if conetype_ is not None:
        _conetype_tmp = (ctypes.c_int32 * len(conetype_))(*conetype_)
    else:
        _conetype_tmp = None
    if conepar_ is None:
      raise ValueError("Argument conepar cannot be None")
    if conepar_ is None:
      raise ValueError("Argument conepar may not be None")
    if isinstance(conepar_, numpy.ndarray) and conepar_.dtype is numpy.dtype(numpy.float64) and conepar_.flags.contiguous:
      _conepar_copyarray = False
      _conepar_tmp = ctypes.cast(conepar_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(conepar_,array.ndarray) and conepar_.dtype is numpy.dtype(array.float64) and  conepar_.getsteplength() == 1:
      _conepar_copyarray = False
      _conepar_tmp = conepar_.getdatacptr()
    elif conepar_ is None:
      _conepar_copyarray = False
      _conepar_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _conepar_copyarray = True
      _conepar_tmp = (ctypes.c_double * len(conepar_))(*conepar_)
    if nummem_ is None:
      raise ValueError("Argument nummem cannot be None")
    if nummem_ is None:
      raise ValueError("Argument nummem may not be None")
    if isinstance(nummem_, numpy.ndarray) and nummem_.dtype is numpy.dtype(numpy.int32) and nummem_.flags.contiguous:
      _nummem_copyarray = False
      _nummem_tmp = ctypes.cast(nummem_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(nummem_,array.ndarray) and nummem_.dtype is numpy.dtype(array.int32) and  nummem_.getsteplength() == 1:
      _nummem_copyarray = False
      _nummem_tmp = nummem_.getdatacptr()
    elif nummem_ is None:
      _nummem_copyarray = False
      _nummem_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _nummem_copyarray = True
      _nummem_tmp = (ctypes.c_int32 * len(nummem_))(*nummem_)
    res = __library__.MSK_XX_appendconesseq(self.__nativep,num_,_conetype_tmp,_conepar_tmp,_nummem_tmp,j_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_make_int,_make_int,_make_double)
  def chgbound(self,accmode_,i_,lower_,finite_,value_):
    """
    Changes the bounds for one constraint or variable.
  
    chgbound(self,accmode_,i_,lower_,finite_,value_)
      accmode: mosek.accmode. Defines if operations are performed row-wise (constraint-oriented) or column-wise (variable-oriented).
      i: int. Index of the constraint or variable for which the bounds should be changed.
      lower: int. If non-zero, then the lower bound is changed, otherwise                             the upper bound is changed.
      finite: int. If non-zero, then the given value is assumed to be finite.
      value: double. New value for the bound.
    """
    res = __library__.MSK_XX_chgbound(self.__nativep,accmode_,i_,lower_,finite_,value_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int)
  def getaij(self,i_,j_):
    """
    Obtains a single coefficient in linear constraint matrix.
  
    getaij(self,i_,j_)
      i: int. Row index of the coefficient to be returned.
      j: int. Column index of the coefficient to be returned.
    returns: aij
      aij: double. Returns the requested coefficient.
    """
    aij_ = ctypes.c_double()
    aij_ = ctypes.c_double()
    res = __library__.MSK_XX_getaij(self.__nativep,i_,j_,ctypes.byref(aij_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _aij_return_value = aij_.value
    return (_aij_return_value)
  @accepts(_accept_any,_make_int,_make_int,_make_int,_make_int)
  def getapiecenumnz(self,firsti_,lasti_,firstj_,lastj_):
    """
    Obtains the number non-zeros in a rectangular piece of the linear constraint matrix.
  
    getapiecenumnz(self,firsti_,lasti_,firstj_,lastj_)
      firsti: int. Index of the first row in the rectangular piece.
      lasti: int. Index of the last row plus one in the rectangular piece.
      firstj: int. Index of the first column in the rectangular piece.
      lastj: int. Index of the last column plus one in the rectangular piece.
    returns: numnz
      numnz: int. Number of non-zero elements in the rectangular piece of the linear constraint matrix.
    """
    numnz_ = ctypes.c_int32()
    numnz_ = ctypes.c_int32()
    res = __library__.MSK_XX_getapiecenumnz(self.__nativep,firsti_,lasti_,firstj_,lastj_,ctypes.byref(numnz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numnz_return_value = numnz_.value
    return (_numnz_return_value)
  @accepts(_accept_any,_make_int)
  def getacolnumnz(self,i_):
    """
    Obtains the number of non-zero elements in one column of the linear constraint matrix
  
    getacolnumnz(self,i_)
      i: int. Index of the column.
    returns: nzj
      nzj: int. Number of non-zeros in the j'th row or column of (A).
    """
    nzj_ = ctypes.c_int32()
    nzj_ = ctypes.c_int32()
    res = __library__.MSK_XX_getacolnumnz(self.__nativep,i_,ctypes.byref(nzj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nzj_return_value = nzj_.value
    return (_nzj_return_value)
  @accepts(_accept_any,_make_int,_accept_intvector,_accept_doublevector)
  def getacol(self,j_,subj_,valj_):
    """
    Obtains one column of the linear constraint matrix.
  
    getacol(self,j_,subj_,valj_)
      j: int. Index of the column.
      subj: array of int. Index of the non-zeros in the column obtained.
      valj: array of double. Numerical values of the column obtained.
    returns: nzj
      nzj: int. Number of non-zeros in the column obtained.
    """
    nzj_ = ctypes.c_int32()
    nzj_ = ctypes.c_int32()
    if subj_ is not None and len(subj_) < self.getacolnumnz((j_)):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if valj_ is not None and len(valj_) < self.getacolnumnz((j_)):
      raise ValueError("Array argument valj is not long enough")
    if isinstance(valj_,numpy.ndarray) and not valj_.flags.writeable:
      raise ValueError("Argument valj must be writable")
    if valj_ is None:
      raise ValueError("Argument valj may not be None")
    if isinstance(valj_, numpy.ndarray) and valj_.dtype is numpy.dtype(numpy.float64) and valj_.flags.contiguous:
      _valj_copyarray = False
      _valj_tmp = ctypes.cast(valj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valj_,array.ndarray) and valj_.dtype is numpy.dtype(array.float64) and  valj_.getsteplength() == 1:
      _valj_copyarray = False
      _valj_tmp = valj_.getdatacptr()
    elif valj_ is None:
      _valj_copyarray = False
      _valj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valj_copyarray = True
      _valj_tmp = (ctypes.c_double * len(valj_))(*valj_)
    res = __library__.MSK_XX_getacol(self.__nativep,j_,ctypes.byref(nzj_),_subj_tmp,_valj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nzj_return_value = nzj_.value
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _valj_copyarray:
      if isinstance(valj_, numpy.ndarray) and False:
        valj_[:] = list(_valj_tmp)
      else:
        valj_[:] = _valj_tmp
    return (_nzj_return_value)
  @accepts(_accept_any,_make_int)
  def getarownumnz(self,i_):
    """
    Obtains the number of non-zero elements in one row of the linear constraint matrix
  
    getarownumnz(self,i_)
      i: int. Index of the row or column.
    returns: nzi
      nzi: int. Number of non-zeros in the i'th row of (A).
    """
    nzi_ = ctypes.c_int32()
    nzi_ = ctypes.c_int32()
    res = __library__.MSK_XX_getarownumnz(self.__nativep,i_,ctypes.byref(nzi_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nzi_return_value = nzi_.value
    return (_nzi_return_value)
  @accepts(_accept_any,_make_int,_accept_intvector,_accept_doublevector)
  def getarow(self,i_,subi_,vali_):
    """
    Obtains one row of the linear constraint matrix.
  
    getarow(self,i_,subi_,vali_)
      i: int. Index of the row or column.
      subi: array of int. Index of the non-zeros in the row obtained.
      vali: array of double. Numerical values of the row obtained.
    returns: nzi
      nzi: int. Number of non-zeros in the row obtained.
    """
    nzi_ = ctypes.c_int32()
    nzi_ = ctypes.c_int32()
    if subi_ is not None and len(subi_) < self.getarownumnz((i_)):
      raise ValueError("Array argument subi is not long enough")
    if isinstance(subi_,numpy.ndarray) and not subi_.flags.writeable:
      raise ValueError("Argument subi must be writable")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if vali_ is not None and len(vali_) < self.getarownumnz((i_)):
      raise ValueError("Array argument vali is not long enough")
    if isinstance(vali_,numpy.ndarray) and not vali_.flags.writeable:
      raise ValueError("Argument vali must be writable")
    if vali_ is None:
      raise ValueError("Argument vali may not be None")
    if isinstance(vali_, numpy.ndarray) and vali_.dtype is numpy.dtype(numpy.float64) and vali_.flags.contiguous:
      _vali_copyarray = False
      _vali_tmp = ctypes.cast(vali_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(vali_,array.ndarray) and vali_.dtype is numpy.dtype(array.float64) and  vali_.getsteplength() == 1:
      _vali_copyarray = False
      _vali_tmp = vali_.getdatacptr()
    elif vali_ is None:
      _vali_copyarray = False
      _vali_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _vali_copyarray = True
      _vali_tmp = (ctypes.c_double * len(vali_))(*vali_)
    res = __library__.MSK_XX_getarow(self.__nativep,i_,ctypes.byref(nzi_),_subi_tmp,_vali_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nzi_return_value = nzi_.value
    if _subi_copyarray:
      if isinstance(subi_, numpy.ndarray) and False:
        subi_[:] = list(_subi_tmp)
      else:
        subi_[:] = _subi_tmp
    if _vali_copyarray:
      if isinstance(vali_, numpy.ndarray) and False:
        vali_[:] = list(_vali_tmp)
      else:
        vali_[:] = _vali_tmp
    return (_nzi_return_value)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_make_int)
  def getaslicenumnz(self,accmode_,first_,last_):
    """
    Obtains the number of non-zeros in a slice of rows or columns of the coefficient matrix.
  
    getaslicenumnz(self,accmode_,first_,last_)
      accmode: mosek.accmode. Defines whether non-zeros are counted in a column slice or a row slice.
      first: int. Index of the first row or column in the sequence.
      last: int. Index of the last row or column plus one in the sequence.
    returns: numnz
      numnz: long. Number of non-zeros in the slice.
    """
    numnz_ = ctypes.c_int64()
    numnz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getaslicenumnz64(self.__nativep,accmode_,first_,last_,ctypes.byref(numnz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numnz_return_value = numnz_.value
    return (_numnz_return_value)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_make_int,_accept_longvector,_accept_longvector,_accept_intvector,_accept_doublevector)
  def getaslice(self,accmode_,first_,last_,ptrb_,ptre_,sub_,val_):
    """
    Obtains a sequence of rows or columns from the coefficient matrix.
  
    getaslice(self,accmode_,first_,last_,ptrb_,ptre_,sub_,val_)
      accmode: mosek.accmode. Defines whether a column slice or a row slice is requested.
      first: int. Index of the first row or column in the sequence.
      last: int. Index of the last row or column in the sequence plus one.
      ptrb: array of long. Row or column start pointers.
      ptre: array of long. Row or column end pointers.
      sub: array of int. Contains the row or column subscripts.
      val: array of double. Contains the coefficient values.
    """
    maxnumnz_ = self.getaslicenumnz((accmode_),(first_),(last_))
    surp_ = ctypes.c_int64(len(sub_))
    if ptrb_ is not None and len(ptrb_) < ((last_) - (first_)):
      raise ValueError("Array argument ptrb is not long enough")
    if isinstance(ptrb_,numpy.ndarray) and not ptrb_.flags.writeable:
      raise ValueError("Argument ptrb must be writable")
    if isinstance(ptrb_, numpy.ndarray) and ptrb_.dtype is numpy.dtype(numpy.int64) and ptrb_.flags.contiguous:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.cast(ptrb_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptrb_,array.ndarray) and ptrb_.dtype is numpy.dtype(array.int64) and  ptrb_.getsteplength() == 1:
      _ptrb_copyarray = False
      _ptrb_tmp = ptrb_.getdatacptr()
    elif ptrb_ is None:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptrb_copyarray = True
      _ptrb_tmp = (ctypes.c_int64 * len(ptrb_))(*ptrb_)
    if ptre_ is not None and len(ptre_) < ((last_) - (first_)):
      raise ValueError("Array argument ptre is not long enough")
    if isinstance(ptre_,numpy.ndarray) and not ptre_.flags.writeable:
      raise ValueError("Argument ptre must be writable")
    if isinstance(ptre_, numpy.ndarray) and ptre_.dtype is numpy.dtype(numpy.int64) and ptre_.flags.contiguous:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.cast(ptre_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptre_,array.ndarray) and ptre_.dtype is numpy.dtype(array.int64) and  ptre_.getsteplength() == 1:
      _ptre_copyarray = False
      _ptre_tmp = ptre_.getdatacptr()
    elif ptre_ is None:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptre_copyarray = True
      _ptre_tmp = (ctypes.c_int64 * len(ptre_))(*ptre_)
    if sub_ is not None and len(sub_) < (maxnumnz_):
      raise ValueError("Array argument sub is not long enough")
    if isinstance(sub_,numpy.ndarray) and not sub_.flags.writeable:
      raise ValueError("Argument sub must be writable")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if val_ is not None and len(val_) < (maxnumnz_):
      raise ValueError("Array argument val is not long enough")
    if isinstance(val_,numpy.ndarray) and not val_.flags.writeable:
      raise ValueError("Argument val must be writable")
    if isinstance(val_, numpy.ndarray) and val_.dtype is numpy.dtype(numpy.float64) and val_.flags.contiguous:
      _val_copyarray = False
      _val_tmp = ctypes.cast(val_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(val_,array.ndarray) and val_.dtype is numpy.dtype(array.float64) and  val_.getsteplength() == 1:
      _val_copyarray = False
      _val_tmp = val_.getdatacptr()
    elif val_ is None:
      _val_copyarray = False
      _val_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _val_copyarray = True
      _val_tmp = (ctypes.c_double * len(val_))(*val_)
    res = __library__.MSK_XX_getaslice64(self.__nativep,accmode_,first_,last_,maxnumnz_,ctypes.byref(surp_),_ptrb_tmp,_ptre_tmp,_sub_tmp,_val_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _ptrb_copyarray:
      if isinstance(ptrb_, numpy.ndarray) and False:
        ptrb_[:] = list(_ptrb_tmp)
      else:
        ptrb_[:] = _ptrb_tmp
    if _ptre_copyarray:
      if isinstance(ptre_, numpy.ndarray) and False:
        ptre_[:] = list(_ptre_tmp)
      else:
        ptre_[:] = _ptre_tmp
    if _sub_copyarray:
      if isinstance(sub_, numpy.ndarray) and False:
        sub_[:] = list(_sub_tmp)
      else:
        sub_[:] = _sub_tmp
    if _val_copyarray:
      if isinstance(val_, numpy.ndarray) and False:
        val_[:] = list(_val_tmp)
      else:
        val_[:] = _val_tmp
  @accepts(_accept_any,_make_int,_make_int,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getarowslicetrip(self,first_,last_,subi_,subj_,val_):
    """
    Obtains a sequence of rows from the coefficient matrix in triplet format.
  
    getarowslicetrip(self,first_,last_,subi_,subj_,val_)
      first: int. Index of the first row or column in the sequence.
      last: int. Index of the last row or column in the sequence plus one.
      subi: array of int. Constraint subscripts.
      subj: array of int. Column subscripts.
      val: array of double. Values.
    """
    maxnumnz_ = None
    if maxnumnz_ is None:
      maxnumnz_ = len(subi_)
    else:
      maxnumnz_ = min(maxnumnz_,len(subi_))
    if maxnumnz_ is None:
      maxnumnz_ = len(subj_)
    else:
      maxnumnz_ = min(maxnumnz_,len(subj_))
    if maxnumnz_ is None:
      maxnumnz_ = len(val_)
    else:
      maxnumnz_ = min(maxnumnz_,len(val_))
    surp_ = ctypes.c_int64(len(subi_))
    if subi_ is not None and len(subi_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument subi is not long enough")
    if isinstance(subi_,numpy.ndarray) and not subi_.flags.writeable:
      raise ValueError("Argument subi must be writable")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is not None and len(subj_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if val_ is not None and len(val_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument val is not long enough")
    if isinstance(val_,numpy.ndarray) and not val_.flags.writeable:
      raise ValueError("Argument val must be writable")
    if isinstance(val_, numpy.ndarray) and val_.dtype is numpy.dtype(numpy.float64) and val_.flags.contiguous:
      _val_copyarray = False
      _val_tmp = ctypes.cast(val_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(val_,array.ndarray) and val_.dtype is numpy.dtype(array.float64) and  val_.getsteplength() == 1:
      _val_copyarray = False
      _val_tmp = val_.getdatacptr()
    elif val_ is None:
      _val_copyarray = False
      _val_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _val_copyarray = True
      _val_tmp = (ctypes.c_double * len(val_))(*val_)
    res = __library__.MSK_XX_getarowslicetrip(self.__nativep,first_,last_,maxnumnz_,ctypes.byref(surp_),_subi_tmp,_subj_tmp,_val_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _subi_copyarray:
      if isinstance(subi_, numpy.ndarray) and False:
        subi_[:] = list(_subi_tmp)
      else:
        subi_[:] = _subi_tmp
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _val_copyarray:
      if isinstance(val_, numpy.ndarray) and False:
        val_[:] = list(_val_tmp)
      else:
        val_[:] = _val_tmp
  @accepts(_accept_any,_make_int,_make_int,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getacolslicetrip(self,first_,last_,subi_,subj_,val_):
    """
    Obtains a sequence of columns from the coefficient matrix in triplet format.
  
    getacolslicetrip(self,first_,last_,subi_,subj_,val_)
      first: int. Index of the first column in the sequence.
      last: int. Index of the last column in the sequence plus one.
      subi: array of int. Constraint subscripts.
      subj: array of int. Column subscripts.
      val: array of double. Values.
    """
    maxnumnz_ = None
    if maxnumnz_ is None:
      maxnumnz_ = len(subi_)
    else:
      maxnumnz_ = min(maxnumnz_,len(subi_))
    if maxnumnz_ is None:
      maxnumnz_ = len(subj_)
    else:
      maxnumnz_ = min(maxnumnz_,len(subj_))
    if maxnumnz_ is None:
      maxnumnz_ = len(val_)
    else:
      maxnumnz_ = min(maxnumnz_,len(val_))
    surp_ = ctypes.c_int64(len(subi_))
    if subi_ is not None and len(subi_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument subi is not long enough")
    if isinstance(subi_,numpy.ndarray) and not subi_.flags.writeable:
      raise ValueError("Argument subi must be writable")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is not None and len(subj_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if val_ is not None and len(val_) < self.getaslicenumnz(accmode.con,(first_),(last_)):
      raise ValueError("Array argument val is not long enough")
    if isinstance(val_,numpy.ndarray) and not val_.flags.writeable:
      raise ValueError("Argument val must be writable")
    if isinstance(val_, numpy.ndarray) and val_.dtype is numpy.dtype(numpy.float64) and val_.flags.contiguous:
      _val_copyarray = False
      _val_tmp = ctypes.cast(val_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(val_,array.ndarray) and val_.dtype is numpy.dtype(array.float64) and  val_.getsteplength() == 1:
      _val_copyarray = False
      _val_tmp = val_.getdatacptr()
    elif val_ is None:
      _val_copyarray = False
      _val_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _val_copyarray = True
      _val_tmp = (ctypes.c_double * len(val_))(*val_)
    res = __library__.MSK_XX_getacolslicetrip(self.__nativep,first_,last_,maxnumnz_,ctypes.byref(surp_),_subi_tmp,_subj_tmp,_val_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _subi_copyarray:
      if isinstance(subi_, numpy.ndarray) and False:
        subi_[:] = list(_subi_tmp)
      else:
        subi_[:] = _subi_tmp
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _val_copyarray:
      if isinstance(val_, numpy.ndarray) and False:
        val_[:] = list(_val_tmp)
      else:
        val_[:] = _val_tmp
  @accepts(_accept_any,_make_int)
  def getconbound(self,i_):
    """
    Obtains bound information for one constraint.
  
    getconbound(self,i_)
      i: int. Index of the constraint for which the bound information should be obtained.
    returns: bk,bl,bu
      bk: mosek.boundkey. Bound keys.
      bl: double. Values for lower bounds.
      bu: double. Values for upper bounds.
    """
    bk_ = ctypes.c_int32()
    bl_ = ctypes.c_double()
    bl_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    res = __library__.MSK_XX_getconbound(self.__nativep,i_,ctypes.byref(bk_),ctypes.byref(bl_),ctypes.byref(bu_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _bk_return_value = boundkey(bk_.value)
    _bl_return_value = bl_.value
    _bu_return_value = bu_.value
    return (_bk_return_value,_bl_return_value,_bu_return_value)
  @accepts(_accept_any,_make_int)
  def getvarbound(self,i_):
    """
    Obtains bound information for one variable.
  
    getvarbound(self,i_)
      i: int. Index of the variable for which the bound information should be obtained.
    returns: bk,bl,bu
      bk: mosek.boundkey. Bound keys.
      bl: double. Values for lower bounds.
      bu: double. Values for upper bounds.
    """
    bk_ = ctypes.c_int32()
    bl_ = ctypes.c_double()
    bl_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    res = __library__.MSK_XX_getvarbound(self.__nativep,i_,ctypes.byref(bk_),ctypes.byref(bl_),ctypes.byref(bu_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _bk_return_value = boundkey(bk_.value)
    _bl_return_value = bl_.value
    _bu_return_value = bu_.value
    return (_bk_return_value,_bl_return_value,_bu_return_value)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int)
  def getbound(self,accmode_,i_):
    """
    Obtains bound information for one constraint or variable.
  
    getbound(self,accmode_,i_)
      accmode: mosek.accmode. Defines if operations are performed row-wise (constraint-oriented) or column-wise (variable-oriented).
      i: int. Index of the constraint or variable for which the bound information should be obtained.
    returns: bk,bl,bu
      bk: mosek.boundkey. Bound keys.
      bl: double. Values for lower bounds.
      bu: double. Values for upper bounds.
    """
    bk_ = ctypes.c_int32()
    bl_ = ctypes.c_double()
    bl_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    bu_ = ctypes.c_double()
    res = __library__.MSK_XX_getbound(self.__nativep,accmode_,i_,ctypes.byref(bk_),ctypes.byref(bl_),ctypes.byref(bu_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _bk_return_value = boundkey(bk_.value)
    _bl_return_value = bl_.value
    _bu_return_value = bu_.value
    return (_bk_return_value,_bl_return_value,_bu_return_value)
  @accepts(_accept_any,_make_int,_make_int,_accept_any,_accept_doublevector,_accept_doublevector)
  def getconboundslice(self,first_,last_,bk_,bl_,bu_):
    """
    Obtains bounds information for a slice of the constraints.
  
    getconboundslice(self,first_,last_,bk_,bl_,bu_)
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      bk: array of mosek.boundkey. Bound keys.
      bl: array of double. Values for lower bounds.
      bu: array of double. Values for upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if isinstance(bk_,numpy.ndarray) and not bk_.flags.writeable:
      raise ValueError("Argument bk must be writable")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))()
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if isinstance(bl_,numpy.ndarray) and not bl_.flags.writeable:
      raise ValueError("Argument bl must be writable")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if isinstance(bu_,numpy.ndarray) and not bu_.flags.writeable:
      raise ValueError("Argument bu must be writable")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_getconboundslice(self.__nativep,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if bk_ is not None: bk_[:] = [ boundkey(v) for v in _bk_tmp[0:len(bk_)] ]
    if _bl_copyarray:
      if isinstance(bl_, numpy.ndarray) and False:
        bl_[:] = list(_bl_tmp)
      else:
        bl_[:] = _bl_tmp
    if _bu_copyarray:
      if isinstance(bu_, numpy.ndarray) and False:
        bu_[:] = list(_bu_tmp)
      else:
        bu_[:] = _bu_tmp
  @accepts(_accept_any,_make_int,_make_int,_accept_any,_accept_doublevector,_accept_doublevector)
  def getvarboundslice(self,first_,last_,bk_,bl_,bu_):
    """
    Obtains bounds information for a slice of the variables.
  
    getvarboundslice(self,first_,last_,bk_,bl_,bu_)
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      bk: array of mosek.boundkey. Bound keys.
      bl: array of double. Values for lower bounds.
      bu: array of double. Values for upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if isinstance(bk_,numpy.ndarray) and not bk_.flags.writeable:
      raise ValueError("Argument bk must be writable")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))()
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if isinstance(bl_,numpy.ndarray) and not bl_.flags.writeable:
      raise ValueError("Argument bl must be writable")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if isinstance(bu_,numpy.ndarray) and not bu_.flags.writeable:
      raise ValueError("Argument bu must be writable")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_getvarboundslice(self.__nativep,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if bk_ is not None: bk_[:] = [ boundkey(v) for v in _bk_tmp[0:len(bk_)] ]
    if _bl_copyarray:
      if isinstance(bl_, numpy.ndarray) and False:
        bl_[:] = list(_bl_tmp)
      else:
        bl_[:] = _bl_tmp
    if _bu_copyarray:
      if isinstance(bu_, numpy.ndarray) and False:
        bu_[:] = list(_bu_tmp)
      else:
        bu_[:] = _bu_tmp
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_make_int,_accept_any,_accept_doublevector,_accept_doublevector)
  def getboundslice(self,accmode_,first_,last_,bk_,bl_,bu_):
    """
    Obtains bounds information for a sequence of variables or constraints.
  
    getboundslice(self,accmode_,first_,last_,bk_,bl_,bu_)
      accmode: mosek.accmode. Defines if operations are performed row-wise (constraint-oriented) or column-wise (variable-oriented).
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      bk: array of mosek.boundkey. Bound keys.
      bl: array of double. Values for lower bounds.
      bu: array of double. Values for upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if isinstance(bk_,numpy.ndarray) and not bk_.flags.writeable:
      raise ValueError("Argument bk must be writable")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))()
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if isinstance(bl_,numpy.ndarray) and not bl_.flags.writeable:
      raise ValueError("Argument bl must be writable")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if isinstance(bu_,numpy.ndarray) and not bu_.flags.writeable:
      raise ValueError("Argument bu must be writable")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_getboundslice(self.__nativep,accmode_,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if bk_ is not None: bk_[:] = [ boundkey(v) for v in _bk_tmp[0:len(bk_)] ]
    if _bl_copyarray:
      if isinstance(bl_, numpy.ndarray) and False:
        bl_[:] = list(_bl_tmp)
      else:
        bl_[:] = _bl_tmp
    if _bu_copyarray:
      if isinstance(bu_, numpy.ndarray) and False:
        bu_[:] = list(_bu_tmp)
      else:
        bu_[:] = _bu_tmp
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_make_int,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putboundslice(self,con_,first_,last_,bk_,bl_,bu_):
    """
    Modifies bounds.
  
    putboundslice(self,con_,first_,last_,bk_,bl_,bu_)
      con: mosek.accmode. Determines whether variables or constraints are modified.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      bk: array of mosek.boundkey. Bound keys.
      bl: array of double. Values for lower bounds.
      bu: array of double. Values for upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if bk_ is None:
      raise ValueError("Argument bk cannot be None")
    if bk_ is None:
      raise ValueError("Argument bk may not be None")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))(*bk_)
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if bl_ is None:
      raise ValueError("Argument bl cannot be None")
    if bl_ is None:
      raise ValueError("Argument bl may not be None")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if bu_ is None:
      raise ValueError("Argument bu cannot be None")
    if bu_ is None:
      raise ValueError("Argument bu may not be None")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_putboundslice(self.__nativep,con_,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def getcj(self,j_):
    """
    Obtains one coefficient of c.
  
    getcj(self,j_)
      j: int. Index of the variable for which c coefficient should be obtained.
    returns: cj
      cj: double. The c coefficient value.
    """
    cj_ = ctypes.c_double()
    cj_ = ctypes.c_double()
    res = __library__.MSK_XX_getcj(self.__nativep,j_,ctypes.byref(cj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _cj_return_value = cj_.value
    return (_cj_return_value)
  @accepts(_accept_any,_accept_doublevector)
  def getc(self,c_):
    """
    Obtains all objective coefficients.
  
    getc(self,c_)
      c: array of double. Linear terms of the objective as a dense vector. The lengths is the number of variables.
    """
    if c_ is not None and len(c_) < self.getnumvar():
      raise ValueError("Array argument c is not long enough")
    if isinstance(c_,numpy.ndarray) and not c_.flags.writeable:
      raise ValueError("Argument c must be writable")
    if isinstance(c_, numpy.ndarray) and c_.dtype is numpy.dtype(numpy.float64) and c_.flags.contiguous:
      _c_copyarray = False
      _c_tmp = ctypes.cast(c_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(c_,array.ndarray) and c_.dtype is numpy.dtype(array.float64) and  c_.getsteplength() == 1:
      _c_copyarray = False
      _c_tmp = c_.getdatacptr()
    elif c_ is None:
      _c_copyarray = False
      _c_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _c_copyarray = True
      _c_tmp = (ctypes.c_double * len(c_))(*c_)
    res = __library__.MSK_XX_getc(self.__nativep,_c_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _c_copyarray:
      if isinstance(c_, numpy.ndarray) and False:
        c_[:] = list(_c_tmp)
      else:
        c_[:] = _c_tmp
  @accepts(_accept_any)
  def getcfix(self):
    """
    Obtains the fixed term in the objective.
  
    getcfix(self)
    returns: cfix
      cfix: double. Fixed term in the objective.
    """
    cfix_ = ctypes.c_double()
    cfix_ = ctypes.c_double()
    res = __library__.MSK_XX_getcfix(self.__nativep,ctypes.byref(cfix_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _cfix_return_value = cfix_.value
    return (_cfix_return_value)
  @accepts(_accept_any,_make_int,_accept_intvector)
  def getcone(self,k_,submem_):
    """
    Obtains a conic constraint.
  
    getcone(self,k_,submem_)
      k: int. Index of the cone constraint.
      submem: array of int. Variable subscripts of the members in the cone.
    returns: conetype,conepar,nummem
      conetype: mosek.conetype. Specifies the type of the cone.
      conepar: double. This argument is currently not used. Can be set to 0.0.
      nummem: int. Number of member variables in the cone.
    """
    conetype_ = ctypes.c_int32()
    conepar_ = ctypes.c_double()
    conepar_ = ctypes.c_double()
    nummem_ = ctypes.c_int32()
    nummem_ = ctypes.c_int32()
    if submem_ is not None and len(submem_) < self.getcone((k_),None)[2]:
      raise ValueError("Array argument submem is not long enough")
    if isinstance(submem_,numpy.ndarray) and not submem_.flags.writeable:
      raise ValueError("Argument submem must be writable")
    if isinstance(submem_, numpy.ndarray) and submem_.dtype is numpy.dtype(numpy.int32) and submem_.flags.contiguous:
      _submem_copyarray = False
      _submem_tmp = ctypes.cast(submem_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(submem_,array.ndarray) and submem_.dtype is numpy.dtype(array.int32) and  submem_.getsteplength() == 1:
      _submem_copyarray = False
      _submem_tmp = submem_.getdatacptr()
    elif submem_ is None:
      _submem_copyarray = False
      _submem_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _submem_copyarray = True
      _submem_tmp = (ctypes.c_int32 * len(submem_))(*submem_)
    res = __library__.MSK_XX_getcone(self.__nativep,k_,ctypes.byref(conetype_),ctypes.byref(conepar_),ctypes.byref(nummem_),_submem_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _conetype_return_value = conetype(conetype_.value)
    _conepar_return_value = conepar_.value
    _nummem_return_value = nummem_.value
    if _submem_copyarray:
      if isinstance(submem_, numpy.ndarray) and False:
        submem_[:] = list(_submem_tmp)
      else:
        submem_[:] = _submem_tmp
    return (_conetype_return_value,_conepar_return_value,_nummem_return_value)
  @accepts(_accept_any,_make_int)
  def getconeinfo(self,k_):
    """
    Obtains information about a conic constraint.
  
    getconeinfo(self,k_)
      k: int. Index of the conic constraint.
    returns: conetype,conepar,nummem
      conetype: mosek.conetype. Specifies the type of the cone.
      conepar: double. This argument is currently not used. Can be set to 0.0.
      nummem: int. Number of member variables in the cone.
    """
    conetype_ = ctypes.c_int32()
    conepar_ = ctypes.c_double()
    conepar_ = ctypes.c_double()
    nummem_ = ctypes.c_int32()
    nummem_ = ctypes.c_int32()
    res = __library__.MSK_XX_getconeinfo(self.__nativep,k_,ctypes.byref(conetype_),ctypes.byref(conepar_),ctypes.byref(nummem_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _conetype_return_value = conetype(conetype_.value)
    _conepar_return_value = conepar_.value
    _nummem_return_value = nummem_.value
    return (_conetype_return_value,_conepar_return_value,_nummem_return_value)
  @accepts(_accept_any,_make_int,_make_int,_accept_doublevector)
  def getcslice(self,first_,last_,c_):
    """
    Obtains a sequence of coefficients from the objective.
  
    getcslice(self,first_,last_,c_)
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      c: array of double. Linear terms of the objective as a dense vector. The lengths is the number of variables.
    """
    if c_ is not None and len(c_) < ((last_) - (first_)):
      raise ValueError("Array argument c is not long enough")
    if isinstance(c_,numpy.ndarray) and not c_.flags.writeable:
      raise ValueError("Argument c must be writable")
    if isinstance(c_, numpy.ndarray) and c_.dtype is numpy.dtype(numpy.float64) and c_.flags.contiguous:
      _c_copyarray = False
      _c_tmp = ctypes.cast(c_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(c_,array.ndarray) and c_.dtype is numpy.dtype(array.float64) and  c_.getsteplength() == 1:
      _c_copyarray = False
      _c_tmp = c_.getdatacptr()
    elif c_ is None:
      _c_copyarray = False
      _c_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _c_copyarray = True
      _c_tmp = (ctypes.c_double * len(c_))(*c_)
    res = __library__.MSK_XX_getcslice(self.__nativep,first_,last_,_c_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _c_copyarray:
      if isinstance(c_, numpy.ndarray) and False:
        c_[:] = list(_c_tmp)
      else:
        c_[:] = _c_tmp
  @accepts(_accept_any,_accept_anyenum(dinfitem))
  def getdouinf(self,whichdinf_):
    """
    Obtains a double information item.
  
    getdouinf(self,whichdinf_)
      whichdinf: mosek.dinfitem. A double float information item.
    returns: dvalue
      dvalue: double. The value of the required double information item.
    """
    dvalue_ = ctypes.c_double()
    dvalue_ = ctypes.c_double()
    res = __library__.MSK_XX_getdouinf(self.__nativep,whichdinf_,ctypes.byref(dvalue_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _dvalue_return_value = dvalue_.value
    return (_dvalue_return_value)
  @accepts(_accept_any,_accept_anyenum(dparam))
  def getdouparam(self,param_):
    """
    Obtains a double parameter.
  
    getdouparam(self,param_)
      param: mosek.dparam. Which parameter.
    returns: parvalue
      parvalue: double. Parameter value.
    """
    parvalue_ = ctypes.c_double()
    parvalue_ = ctypes.c_double()
    res = __library__.MSK_XX_getdouparam(self.__nativep,param_,ctypes.byref(parvalue_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _parvalue_return_value = parvalue_.value
    return (_parvalue_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getdualobj(self,whichsol_):
    """
    Computes the dual objective value associated with the solution.
  
    getdualobj(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: dualobj
      dualobj: double. Objective value corresponding to the dual solution.
    """
    dualobj_ = ctypes.c_double()
    dualobj_ = ctypes.c_double()
    res = __library__.MSK_XX_getdualobj(self.__nativep,whichsol_,ctypes.byref(dualobj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _dualobj_return_value = dualobj_.value
    return (_dualobj_return_value)
  @accepts(_accept_any,_accept_anyenum(iinfitem))
  def getintinf(self,whichiinf_):
    """
    Obtains an integer information item.
  
    getintinf(self,whichiinf_)
      whichiinf: mosek.iinfitem. Specifies an information item.
    returns: ivalue
      ivalue: int. The value of the required integer information item.
    """
    ivalue_ = ctypes.c_int32()
    ivalue_ = ctypes.c_int32()
    res = __library__.MSK_XX_getintinf(self.__nativep,whichiinf_,ctypes.byref(ivalue_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _ivalue_return_value = ivalue_.value
    return (_ivalue_return_value)
  @accepts(_accept_any,_accept_anyenum(liinfitem))
  def getlintinf(self,whichliinf_):
    """
    Obtains an integer information item.
  
    getlintinf(self,whichliinf_)
      whichliinf: mosek.liinfitem. Specifies an information item.
    returns: ivalue
      ivalue: long. The value of the required integer information item.
    """
    ivalue_ = ctypes.c_int64()
    ivalue_ = ctypes.c_int64()
    res = __library__.MSK_XX_getlintinf(self.__nativep,whichliinf_,ctypes.byref(ivalue_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _ivalue_return_value = ivalue_.value
    return (_ivalue_return_value)
  @accepts(_accept_any,_accept_anyenum(iparam))
  def getintparam(self,param_):
    """
    Obtains an integer parameter.
  
    getintparam(self,param_)
      param: mosek.iparam. Which parameter.
    returns: parvalue
      parvalue: int. Parameter value.
    """
    parvalue_ = ctypes.c_int32()
    parvalue_ = ctypes.c_int32()
    res = __library__.MSK_XX_getintparam(self.__nativep,param_,ctypes.byref(parvalue_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _parvalue_return_value = parvalue_.value
    return (_parvalue_return_value)
  @accepts(_accept_any)
  def getmaxnumanz(self):
    """
    Obtains number of preallocated non-zeros in the linear constraint matrix.
  
    getmaxnumanz(self)
    returns: maxnumanz
      maxnumanz: long. Number of preallocated non-zero linear matrix elements.
    """
    maxnumanz_ = ctypes.c_int64()
    maxnumanz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getmaxnumanz64(self.__nativep,ctypes.byref(maxnumanz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumanz_return_value = maxnumanz_.value
    return (_maxnumanz_return_value)
  @accepts(_accept_any)
  def getmaxnumcon(self):
    """
    Obtains the number of preallocated constraints in the optimization task.
  
    getmaxnumcon(self)
    returns: maxnumcon
      maxnumcon: int. Number of preallocated constraints in the optimization task.
    """
    maxnumcon_ = ctypes.c_int32()
    maxnumcon_ = ctypes.c_int32()
    res = __library__.MSK_XX_getmaxnumcon(self.__nativep,ctypes.byref(maxnumcon_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumcon_return_value = maxnumcon_.value
    return (_maxnumcon_return_value)
  @accepts(_accept_any)
  def getmaxnumvar(self):
    """
    Obtains the maximum number variables allowed.
  
    getmaxnumvar(self)
    returns: maxnumvar
      maxnumvar: int. Number of preallocated variables in the optimization task.
    """
    maxnumvar_ = ctypes.c_int32()
    maxnumvar_ = ctypes.c_int32()
    res = __library__.MSK_XX_getmaxnumvar(self.__nativep,ctypes.byref(maxnumvar_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumvar_return_value = maxnumvar_.value
    return (_maxnumvar_return_value)
  @accepts(_accept_any,_make_int)
  def getbarvarnamelen(self,i_):
    """
    Obtains the length of a name of a semidefinite variable.
  
    getbarvarnamelen(self,i_)
      i: int. Index.
    returns: len
      len: int. Returns the length of the indicated name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getbarvarnamelen(self.__nativep,i_,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any,_make_int)
  def getbarvarname(self,i_):
    """
    Obtains a name of a semidefinite variable.
  
    getbarvarname(self,i_)
      i: int. Index.
    returns: name
      name: unicode. The requested name is copied to this buffer.
    """
    maxlen_ = (1 + self.getbarvarnamelen((i_)))
    name_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getbarvarname(self.__nativep,i_,maxlen_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _name_retval = name_.value.decode("utf-8")
    return (_name_retval)
  @accepts(_accept_any,_accept_str)
  def getbarvarnameindex(self,somename_):
    """
    Obtains the index of name of semidefinite variable.
  
    getbarvarnameindex(self,somename_)
      somename: str|unicode. The requested name is copied to this buffer.
    returns: asgn,index
      asgn: int. Is non-zero if name somename is assigned to a semidefinite variable.
      index: int. If the name somename is assigned to a semidefinite variable, then index is the name of the constraint.
    """
    if isinstance(somename_,unicode):
      somename_ = somename_.encode("utf-8")
    asgn_ = ctypes.c_int32()
    asgn_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    res = __library__.MSK_XX_getbarvarnameindex(self.__nativep,somename_,ctypes.byref(asgn_),ctypes.byref(index_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _asgn_return_value = asgn_.value
    _index_return_value = index_.value
    return (_asgn_return_value,_index_return_value)
  @accepts(_accept_any,_make_int,_accept_str)
  def putconname(self,i_,name_):
    """
    Puts the name of a constraint.
  
    putconname(self,i_,name_)
      i: int. Index of the variable.
      name: str|unicode. The variable name.
    """
    if isinstance(name_,unicode):
      name_ = name_.encode("utf-8")
    res = __library__.MSK_XX_putconname(self.__nativep,i_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_str)
  def putvarname(self,j_,name_):
    """
    Puts the name of a variable.
  
    putvarname(self,j_,name_)
      j: int. Index of the variable.
      name: str|unicode. The variable name.
    """
    if isinstance(name_,unicode):
      name_ = name_.encode("utf-8")
    res = __library__.MSK_XX_putvarname(self.__nativep,j_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_str)
  def putconename(self,j_,name_):
    """
    Puts the name of a cone.
  
    putconename(self,j_,name_)
      j: int. Index of the variable.
      name: str|unicode. The variable name.
    """
    if isinstance(name_,unicode):
      name_ = name_.encode("utf-8")
    res = __library__.MSK_XX_putconename(self.__nativep,j_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_str)
  def putbarvarname(self,j_,name_):
    """
    Puts the name of a semidefinite variable.
  
    putbarvarname(self,j_,name_)
      j: int. Index of the variable.
      name: str|unicode. The variable name.
    """
    if isinstance(name_,unicode):
      name_ = name_.encode("utf-8")
    res = __library__.MSK_XX_putbarvarname(self.__nativep,j_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def getvarnamelen(self,i_):
    """
    Obtains the length of a name of a variable variable.
  
    getvarnamelen(self,i_)
      i: int. Index.
    returns: len
      len: int. Returns the length of the indicated name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getvarnamelen(self.__nativep,i_,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any,_make_int)
  def getvarname(self,j_):
    """
    Obtains a name of a variable.
  
    getvarname(self,j_)
      j: int. Index.
    returns: name
      name: unicode. Is assigned the required name.
    """
    maxlen_ = (1 + self.getvarnamelen((j_)))
    name_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getvarname(self.__nativep,j_,maxlen_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _name_retval = name_.value.decode("utf-8")
    return (_name_retval)
  @accepts(_accept_any,_make_int)
  def getconnamelen(self,i_):
    """
    Obtains the length of a name of a constraint variable.
  
    getconnamelen(self,i_)
      i: int. Index.
    returns: len
      len: int. Returns the length of the indicated name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getconnamelen(self.__nativep,i_,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any,_make_int)
  def getconname(self,i_):
    """
    Obtains a name of a constraint.
  
    getconname(self,i_)
      i: int. Index.
    returns: name
      name: unicode. Is assigned the required name.
    """
    maxlen_ = (1 + self.getconnamelen((i_)))
    name_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getconname(self.__nativep,i_,maxlen_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _name_retval = name_.value.decode("utf-8")
    return (_name_retval)
  @accepts(_accept_any,_accept_str)
  def getconnameindex(self,somename_):
    """
    Checks whether the name somename has been assigned to any constraint.
  
    getconnameindex(self,somename_)
      somename: str|unicode. The name which should be checked.
    returns: asgn,index
      asgn: int. Is non-zero if name somename is assigned to a constraint.
      index: int. If the name somename is assigned to a constraint, then index is the name of the constraint.
    """
    if isinstance(somename_,unicode):
      somename_ = somename_.encode("utf-8")
    asgn_ = ctypes.c_int32()
    asgn_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    res = __library__.MSK_XX_getconnameindex(self.__nativep,somename_,ctypes.byref(asgn_),ctypes.byref(index_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _asgn_return_value = asgn_.value
    _index_return_value = index_.value
    return (_asgn_return_value,_index_return_value)
  @accepts(_accept_any,_accept_str)
  def getvarnameindex(self,somename_):
    """
    Checks whether the name somename has been assigned to any variable.
  
    getvarnameindex(self,somename_)
      somename: str|unicode. The name which should be checked.
    returns: asgn,index
      asgn: int. Is non-zero if name somename is assigned to a variable.
      index: int. If the name somename is assigned to a variable, then index is the name of the variable.
    """
    if isinstance(somename_,unicode):
      somename_ = somename_.encode("utf-8")
    asgn_ = ctypes.c_int32()
    asgn_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    res = __library__.MSK_XX_getvarnameindex(self.__nativep,somename_,ctypes.byref(asgn_),ctypes.byref(index_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _asgn_return_value = asgn_.value
    _index_return_value = index_.value
    return (_asgn_return_value,_index_return_value)
  @accepts(_accept_any,_make_int)
  def getconenamelen(self,i_):
    """
    Obtains the length of a name of a cone.
  
    getconenamelen(self,i_)
      i: int. Index.
    returns: len
      len: int. Returns the length of the indicated name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getconenamelen(self.__nativep,i_,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any,_make_int)
  def getconename(self,i_):
    """
    Obtains a name of a cone.
  
    getconename(self,i_)
      i: int. Index.
    returns: name
      name: unicode. Is assigned the required name.
    """
    maxlen_ = (1 + self.getconenamelen((i_)))
    name_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getconename(self.__nativep,i_,maxlen_,name_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _name_retval = name_.value.decode("utf-8")
    return (_name_retval)
  @accepts(_accept_any,_accept_str)
  def getconenameindex(self,somename_):
    """
    Checks whether the name somename has been assigned to any cone.
  
    getconenameindex(self,somename_)
      somename: str|unicode. The name which should be checked.
    returns: asgn,index
      asgn: int. Is non-zero if name somename is assigned to a cone.
      index: int. If the name somename is assigned to a cone, then index is the name of the cone.
    """
    if isinstance(somename_,unicode):
      somename_ = somename_.encode("utf-8")
    asgn_ = ctypes.c_int32()
    asgn_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    index_ = ctypes.c_int32()
    res = __library__.MSK_XX_getconenameindex(self.__nativep,somename_,ctypes.byref(asgn_),ctypes.byref(index_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _asgn_return_value = asgn_.value
    _index_return_value = index_.value
    return (_asgn_return_value,_index_return_value)
  @accepts(_accept_any)
  def getnumanz(self):
    """
    Obtains the number of non-zeros in the coefficient matrix.
  
    getnumanz(self)
    returns: numanz
      numanz: int. Number of non-zero elements in the linear constraint matrix.
    """
    numanz_ = ctypes.c_int32()
    numanz_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumanz(self.__nativep,ctypes.byref(numanz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numanz_return_value = numanz_.value
    return (_numanz_return_value)
  @accepts(_accept_any)
  def getnumanz64(self):
    """
    Obtains the number of non-zeros in the coefficient matrix.
  
    getnumanz64(self)
    returns: numanz
      numanz: long. Number of non-zero elements in the linear constraint matrix.
    """
    numanz_ = ctypes.c_int64()
    numanz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumanz64(self.__nativep,ctypes.byref(numanz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numanz_return_value = numanz_.value
    return (_numanz_return_value)
  @accepts(_accept_any)
  def getnumcon(self):
    """
    Obtains the number of constraints.
  
    getnumcon(self)
    returns: numcon
      numcon: int. Number of constraints.
    """
    numcon_ = ctypes.c_int32()
    numcon_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumcon(self.__nativep,ctypes.byref(numcon_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numcon_return_value = numcon_.value
    return (_numcon_return_value)
  @accepts(_accept_any)
  def getnumcone(self):
    """
    Obtains the number of cones.
  
    getnumcone(self)
    returns: numcone
      numcone: int. Number conic constraints.
    """
    numcone_ = ctypes.c_int32()
    numcone_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumcone(self.__nativep,ctypes.byref(numcone_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numcone_return_value = numcone_.value
    return (_numcone_return_value)
  @accepts(_accept_any,_make_int)
  def getnumconemem(self,k_):
    """
    Obtains the number of members in a cone.
  
    getnumconemem(self,k_)
      k: int. Index of the cone.
    returns: nummem
      nummem: int. Number of member variables in the cone.
    """
    nummem_ = ctypes.c_int32()
    nummem_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumconemem(self.__nativep,k_,ctypes.byref(nummem_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nummem_return_value = nummem_.value
    return (_nummem_return_value)
  @accepts(_accept_any)
  def getnumintvar(self):
    """
    Obtains the number of integer-constrained variables.
  
    getnumintvar(self)
    returns: numintvar
      numintvar: int. Number of integer variables.
    """
    numintvar_ = ctypes.c_int32()
    numintvar_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumintvar(self.__nativep,ctypes.byref(numintvar_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numintvar_return_value = numintvar_.value
    return (_numintvar_return_value)
  @accepts(_accept_any,_accept_anyenum(parametertype))
  def getnumparam(self,partype_):
    """
    Obtains the number of parameters of a given type.
  
    getnumparam(self,partype_)
      partype: mosek.parametertype. Parameter type.
    returns: numparam
      numparam: int. Returns the number of parameters of the requested type.
    """
    numparam_ = ctypes.c_int32()
    numparam_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumparam(self.__nativep,partype_,ctypes.byref(numparam_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numparam_return_value = numparam_.value
    return (_numparam_return_value)
  @accepts(_accept_any,_make_int)
  def getnumqconknz(self,k_):
    """
    Obtains the number of non-zero quadratic terms in a constraint.
  
    getnumqconknz(self,k_)
      k: int. Index of the constraint for which the number of non-zero quadratic terms should be obtained.
    returns: numqcnz
      numqcnz: int. Number of quadratic terms.
    """
    numqcnz_ = ctypes.c_int32()
    numqcnz_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumqconknz(self.__nativep,k_,ctypes.byref(numqcnz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqcnz_return_value = numqcnz_.value
    return (_numqcnz_return_value)
  @accepts(_accept_any,_make_int)
  def getnumqconknz64(self,k_):
    """
    Obtains the number of non-zero quadratic terms in a constraint.
  
    getnumqconknz64(self,k_)
      k: int. Index of the constraint for which the number quadratic terms should be obtained.
    returns: numqcnz
      numqcnz: long. Number of quadratic terms.
    """
    numqcnz_ = ctypes.c_int64()
    numqcnz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumqconknz64(self.__nativep,k_,ctypes.byref(numqcnz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqcnz_return_value = numqcnz_.value
    return (_numqcnz_return_value)
  @accepts(_accept_any)
  def getnumqobjnz(self):
    """
    Obtains the number of non-zero quadratic terms in the objective.
  
    getnumqobjnz(self)
    returns: numqonz
      numqonz: long. Number of non-zero elements in the quadratic objective terms.
    """
    numqonz_ = ctypes.c_int64()
    numqonz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumqobjnz64(self.__nativep,ctypes.byref(numqonz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqonz_return_value = numqonz_.value
    return (_numqonz_return_value)
  @accepts(_accept_any)
  def getnumvar(self):
    """
    Obtains the number of variables.
  
    getnumvar(self)
    returns: numvar
      numvar: int. Number of variables.
    """
    numvar_ = ctypes.c_int32()
    numvar_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumvar(self.__nativep,ctypes.byref(numvar_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numvar_return_value = numvar_.value
    return (_numvar_return_value)
  @accepts(_accept_any)
  def getnumbarvar(self):
    """
    Obtains the number of semidefinite variables.
  
    getnumbarvar(self)
    returns: numbarvar
      numbarvar: int. Number of semidefinite variable in the problem.
    """
    numbarvar_ = ctypes.c_int32()
    numbarvar_ = ctypes.c_int32()
    res = __library__.MSK_XX_getnumbarvar(self.__nativep,ctypes.byref(numbarvar_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numbarvar_return_value = numbarvar_.value
    return (_numbarvar_return_value)
  @accepts(_accept_any)
  def getmaxnumbarvar(self):
    """
    Obtains the number of semidefinite variables.
  
    getmaxnumbarvar(self)
    returns: maxnumbarvar
      maxnumbarvar: int. Obtains maximum number of semidefinite variable currently allowed.
    """
    maxnumbarvar_ = ctypes.c_int32()
    maxnumbarvar_ = ctypes.c_int32()
    res = __library__.MSK_XX_getmaxnumbarvar(self.__nativep,ctypes.byref(maxnumbarvar_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumbarvar_return_value = maxnumbarvar_.value
    return (_maxnumbarvar_return_value)
  @accepts(_accept_any,_make_int)
  def getdimbarvarj(self,j_):
    """
    Obtains the dimension of a symmetric matrix variable.
  
    getdimbarvarj(self,j_)
      j: int. Index of the semidefinite variable whose dimension is requested.
    returns: dimbarvarj
      dimbarvarj: int. The dimension of the j'th semidefinite variable.
    """
    dimbarvarj_ = ctypes.c_int32()
    dimbarvarj_ = ctypes.c_int32()
    res = __library__.MSK_XX_getdimbarvarj(self.__nativep,j_,ctypes.byref(dimbarvarj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _dimbarvarj_return_value = dimbarvarj_.value
    return (_dimbarvarj_return_value)
  @accepts(_accept_any,_make_int)
  def getlenbarvarj(self,j_):
    """
    Obtains the length if the j'th semidefinite variables.
  
    getlenbarvarj(self,j_)
      j: int. Index of the semidefinite variable whose length if requested.
    returns: lenbarvarj
      lenbarvarj: long. Number of scalar elements in the lower triangular part of the semidefinite variable.
    """
    lenbarvarj_ = ctypes.c_int64()
    lenbarvarj_ = ctypes.c_int64()
    res = __library__.MSK_XX_getlenbarvarj(self.__nativep,j_,ctypes.byref(lenbarvarj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _lenbarvarj_return_value = lenbarvarj_.value
    return (_lenbarvarj_return_value)
  @accepts(_accept_any)
  def getobjname(self):
    """
    Obtains the name assigned to the objective function.
  
    getobjname(self)
    returns: objname
      objname: unicode. Assigned the objective name.
    """
    maxlen_ = (1 + self.getobjnamelen())
    objname_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getobjname(self.__nativep,maxlen_,objname_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _objname_retval = objname_.value.decode("utf-8")
    return (_objname_retval)
  @accepts(_accept_any)
  def getobjnamelen(self):
    """
    Obtains the length of the name assigned to the objective function.
  
    getobjnamelen(self)
    returns: len
      len: int. Assigned the length of the objective name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getobjnamelen(self.__nativep,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getprimalobj(self,whichsol_):
    """
    Computes the primal objective value for the desired solution.
  
    getprimalobj(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: primalobj
      primalobj: double. Objective value corresponding to the primal solution.
    """
    primalobj_ = ctypes.c_double()
    primalobj_ = ctypes.c_double()
    res = __library__.MSK_XX_getprimalobj(self.__nativep,whichsol_,ctypes.byref(primalobj_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _primalobj_return_value = primalobj_.value
    return (_primalobj_return_value)
  @accepts(_accept_any)
  def getprobtype(self):
    """
    Obtains the problem type.
  
    getprobtype(self)
    returns: probtype
      probtype: mosek.problemtype. The problem type.
    """
    probtype_ = ctypes.c_int32()
    res = __library__.MSK_XX_getprobtype(self.__nativep,ctypes.byref(probtype_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _probtype_return_value = problemtype(probtype_.value)
    return (_probtype_return_value)
  @accepts(_accept_any,_make_int,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getqconk(self,k_,qcsubi_,qcsubj_,qcval_):
    """
    Obtains all the quadratic terms in a constraint.
  
    getqconk(self,k_,qcsubi_,qcsubj_,qcval_)
      k: int. Which constraint.
      qcsubi: array of int. Row subscripts for quadratic constraint matrix.
      qcsubj: array of int. Column subscripts for quadratic constraint matrix.
      qcval: array of double. Quadratic constraint coefficient values.
    returns: numqcnz
      numqcnz: long. Number of quadratic terms.
    """
    maxnumqcnz_ = self.getnumqconknz((k_))
    qcsurp_ = ctypes.c_int64(len(qcsubi_))
    numqcnz_ = ctypes.c_int64()
    numqcnz_ = ctypes.c_int64()
    if qcsubi_ is not None and len(qcsubi_) < (maxnumqcnz_):
      raise ValueError("Array argument qcsubi is not long enough")
    if isinstance(qcsubi_,numpy.ndarray) and not qcsubi_.flags.writeable:
      raise ValueError("Argument qcsubi must be writable")
    if qcsubi_ is None:
      raise ValueError("Argument qcsubi may not be None")
    if isinstance(qcsubi_, numpy.ndarray) and qcsubi_.dtype is numpy.dtype(numpy.int32) and qcsubi_.flags.contiguous:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.cast(qcsubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubi_,array.ndarray) and qcsubi_.dtype is numpy.dtype(array.int32) and  qcsubi_.getsteplength() == 1:
      _qcsubi_copyarray = False
      _qcsubi_tmp = qcsubi_.getdatacptr()
    elif qcsubi_ is None:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubi_copyarray = True
      _qcsubi_tmp = (ctypes.c_int32 * len(qcsubi_))(*qcsubi_)
    if qcsubj_ is not None and len(qcsubj_) < (maxnumqcnz_):
      raise ValueError("Array argument qcsubj is not long enough")
    if isinstance(qcsubj_,numpy.ndarray) and not qcsubj_.flags.writeable:
      raise ValueError("Argument qcsubj must be writable")
    if qcsubj_ is None:
      raise ValueError("Argument qcsubj may not be None")
    if isinstance(qcsubj_, numpy.ndarray) and qcsubj_.dtype is numpy.dtype(numpy.int32) and qcsubj_.flags.contiguous:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.cast(qcsubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubj_,array.ndarray) and qcsubj_.dtype is numpy.dtype(array.int32) and  qcsubj_.getsteplength() == 1:
      _qcsubj_copyarray = False
      _qcsubj_tmp = qcsubj_.getdatacptr()
    elif qcsubj_ is None:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubj_copyarray = True
      _qcsubj_tmp = (ctypes.c_int32 * len(qcsubj_))(*qcsubj_)
    if qcval_ is not None and len(qcval_) < (maxnumqcnz_):
      raise ValueError("Array argument qcval is not long enough")
    if isinstance(qcval_,numpy.ndarray) and not qcval_.flags.writeable:
      raise ValueError("Argument qcval must be writable")
    if qcval_ is None:
      raise ValueError("Argument qcval may not be None")
    if isinstance(qcval_, numpy.ndarray) and qcval_.dtype is numpy.dtype(numpy.float64) and qcval_.flags.contiguous:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.cast(qcval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qcval_,array.ndarray) and qcval_.dtype is numpy.dtype(array.float64) and  qcval_.getsteplength() == 1:
      _qcval_copyarray = False
      _qcval_tmp = qcval_.getdatacptr()
    elif qcval_ is None:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qcval_copyarray = True
      _qcval_tmp = (ctypes.c_double * len(qcval_))(*qcval_)
    res = __library__.MSK_XX_getqconk64(self.__nativep,k_,maxnumqcnz_,ctypes.byref(qcsurp_),ctypes.byref(numqcnz_),_qcsubi_tmp,_qcsubj_tmp,_qcval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqcnz_return_value = numqcnz_.value
    if _qcsubi_copyarray:
      if isinstance(qcsubi_, numpy.ndarray) and False:
        qcsubi_[:] = list(_qcsubi_tmp)
      else:
        qcsubi_[:] = _qcsubi_tmp
    if _qcsubj_copyarray:
      if isinstance(qcsubj_, numpy.ndarray) and False:
        qcsubj_[:] = list(_qcsubj_tmp)
      else:
        qcsubj_[:] = _qcsubj_tmp
    if _qcval_copyarray:
      if isinstance(qcval_, numpy.ndarray) and False:
        qcval_[:] = list(_qcval_tmp)
      else:
        qcval_[:] = _qcval_tmp
    return (_numqcnz_return_value)
  @accepts(_accept_any,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getqobj(self,qosubi_,qosubj_,qoval_):
    """
    Obtains all the quadratic terms in the objective.
  
    getqobj(self,qosubi_,qosubj_,qoval_)
      qosubi: array of int. Row subscripts for quadratic objective coefficients.
      qosubj: array of int. Column subscripts for quadratic objective coefficients.
      qoval: array of double. Quadratic objective coefficient values.
    returns: numqonz
      numqonz: int. Number of non-zero elements in the quadratic objective terms.
    """
    maxnumqonz_ = self.getnumqobjnz()
    qosurp_ = ctypes.c_int32(len(qosubi_))
    numqonz_ = ctypes.c_int32()
    numqonz_ = ctypes.c_int32()
    if qosubi_ is not None and len(qosubi_) < (maxnumqonz_):
      raise ValueError("Array argument qosubi is not long enough")
    if isinstance(qosubi_,numpy.ndarray) and not qosubi_.flags.writeable:
      raise ValueError("Argument qosubi must be writable")
    if qosubi_ is None:
      raise ValueError("Argument qosubi may not be None")
    if isinstance(qosubi_, numpy.ndarray) and qosubi_.dtype is numpy.dtype(numpy.int32) and qosubi_.flags.contiguous:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.cast(qosubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubi_,array.ndarray) and qosubi_.dtype is numpy.dtype(array.int32) and  qosubi_.getsteplength() == 1:
      _qosubi_copyarray = False
      _qosubi_tmp = qosubi_.getdatacptr()
    elif qosubi_ is None:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubi_copyarray = True
      _qosubi_tmp = (ctypes.c_int32 * len(qosubi_))(*qosubi_)
    if qosubj_ is not None and len(qosubj_) < (maxnumqonz_):
      raise ValueError("Array argument qosubj is not long enough")
    if isinstance(qosubj_,numpy.ndarray) and not qosubj_.flags.writeable:
      raise ValueError("Argument qosubj must be writable")
    if qosubj_ is None:
      raise ValueError("Argument qosubj may not be None")
    if isinstance(qosubj_, numpy.ndarray) and qosubj_.dtype is numpy.dtype(numpy.int32) and qosubj_.flags.contiguous:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.cast(qosubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubj_,array.ndarray) and qosubj_.dtype is numpy.dtype(array.int32) and  qosubj_.getsteplength() == 1:
      _qosubj_copyarray = False
      _qosubj_tmp = qosubj_.getdatacptr()
    elif qosubj_ is None:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubj_copyarray = True
      _qosubj_tmp = (ctypes.c_int32 * len(qosubj_))(*qosubj_)
    if qoval_ is not None and len(qoval_) < (maxnumqonz_):
      raise ValueError("Array argument qoval is not long enough")
    if isinstance(qoval_,numpy.ndarray) and not qoval_.flags.writeable:
      raise ValueError("Argument qoval must be writable")
    if qoval_ is None:
      raise ValueError("Argument qoval may not be None")
    if isinstance(qoval_, numpy.ndarray) and qoval_.dtype is numpy.dtype(numpy.float64) and qoval_.flags.contiguous:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.cast(qoval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qoval_,array.ndarray) and qoval_.dtype is numpy.dtype(array.float64) and  qoval_.getsteplength() == 1:
      _qoval_copyarray = False
      _qoval_tmp = qoval_.getdatacptr()
    elif qoval_ is None:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qoval_copyarray = True
      _qoval_tmp = (ctypes.c_double * len(qoval_))(*qoval_)
    res = __library__.MSK_XX_getqobj(self.__nativep,maxnumqonz_,ctypes.byref(qosurp_),ctypes.byref(numqonz_),_qosubi_tmp,_qosubj_tmp,_qoval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqonz_return_value = numqonz_.value
    if _qosubi_copyarray:
      if isinstance(qosubi_, numpy.ndarray) and False:
        qosubi_[:] = list(_qosubi_tmp)
      else:
        qosubi_[:] = _qosubi_tmp
    if _qosubj_copyarray:
      if isinstance(qosubj_, numpy.ndarray) and False:
        qosubj_[:] = list(_qosubj_tmp)
      else:
        qosubj_[:] = _qosubj_tmp
    if _qoval_copyarray:
      if isinstance(qoval_, numpy.ndarray) and False:
        qoval_[:] = list(_qoval_tmp)
      else:
        qoval_[:] = _qoval_tmp
    return (_numqonz_return_value)
  @accepts(_accept_any,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getqobj64(self,qosubi_,qosubj_,qoval_):
    """
    Obtains all the quadratic terms in the objective.
  
    getqobj64(self,qosubi_,qosubj_,qoval_)
      qosubi: array of int. Row subscripts for quadratic objective coefficients.
      qosubj: array of int. Column subscripts for quadratic objective coefficients.
      qoval: array of double. Quadratic objective coefficient values.
    returns: numqonz
      numqonz: long. Number of non-zero elements in the quadratic objective terms.
    """
    maxnumqonz_ = self.getnumqobjnz()
    qosurp_ = ctypes.c_int64(len(qosubi_))
    numqonz_ = ctypes.c_int64()
    numqonz_ = ctypes.c_int64()
    if qosubi_ is not None and len(qosubi_) < (maxnumqonz_):
      raise ValueError("Array argument qosubi is not long enough")
    if isinstance(qosubi_,numpy.ndarray) and not qosubi_.flags.writeable:
      raise ValueError("Argument qosubi must be writable")
    if qosubi_ is None:
      raise ValueError("Argument qosubi may not be None")
    if isinstance(qosubi_, numpy.ndarray) and qosubi_.dtype is numpy.dtype(numpy.int32) and qosubi_.flags.contiguous:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.cast(qosubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubi_,array.ndarray) and qosubi_.dtype is numpy.dtype(array.int32) and  qosubi_.getsteplength() == 1:
      _qosubi_copyarray = False
      _qosubi_tmp = qosubi_.getdatacptr()
    elif qosubi_ is None:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubi_copyarray = True
      _qosubi_tmp = (ctypes.c_int32 * len(qosubi_))(*qosubi_)
    if qosubj_ is not None and len(qosubj_) < (maxnumqonz_):
      raise ValueError("Array argument qosubj is not long enough")
    if isinstance(qosubj_,numpy.ndarray) and not qosubj_.flags.writeable:
      raise ValueError("Argument qosubj must be writable")
    if qosubj_ is None:
      raise ValueError("Argument qosubj may not be None")
    if isinstance(qosubj_, numpy.ndarray) and qosubj_.dtype is numpy.dtype(numpy.int32) and qosubj_.flags.contiguous:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.cast(qosubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubj_,array.ndarray) and qosubj_.dtype is numpy.dtype(array.int32) and  qosubj_.getsteplength() == 1:
      _qosubj_copyarray = False
      _qosubj_tmp = qosubj_.getdatacptr()
    elif qosubj_ is None:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubj_copyarray = True
      _qosubj_tmp = (ctypes.c_int32 * len(qosubj_))(*qosubj_)
    if qoval_ is not None and len(qoval_) < (maxnumqonz_):
      raise ValueError("Array argument qoval is not long enough")
    if isinstance(qoval_,numpy.ndarray) and not qoval_.flags.writeable:
      raise ValueError("Argument qoval must be writable")
    if qoval_ is None:
      raise ValueError("Argument qoval may not be None")
    if isinstance(qoval_, numpy.ndarray) and qoval_.dtype is numpy.dtype(numpy.float64) and qoval_.flags.contiguous:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.cast(qoval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qoval_,array.ndarray) and qoval_.dtype is numpy.dtype(array.float64) and  qoval_.getsteplength() == 1:
      _qoval_copyarray = False
      _qoval_tmp = qoval_.getdatacptr()
    elif qoval_ is None:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qoval_copyarray = True
      _qoval_tmp = (ctypes.c_double * len(qoval_))(*qoval_)
    res = __library__.MSK_XX_getqobj64(self.__nativep,maxnumqonz_,ctypes.byref(qosurp_),ctypes.byref(numqonz_),_qosubi_tmp,_qosubj_tmp,_qoval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numqonz_return_value = numqonz_.value
    if _qosubi_copyarray:
      if isinstance(qosubi_, numpy.ndarray) and False:
        qosubi_[:] = list(_qosubi_tmp)
      else:
        qosubi_[:] = _qosubi_tmp
    if _qosubj_copyarray:
      if isinstance(qosubj_, numpy.ndarray) and False:
        qosubj_[:] = list(_qosubj_tmp)
      else:
        qosubj_[:] = _qosubj_tmp
    if _qoval_copyarray:
      if isinstance(qoval_, numpy.ndarray) and False:
        qoval_[:] = list(_qoval_tmp)
      else:
        qoval_[:] = _qoval_tmp
    return (_numqonz_return_value)
  @accepts(_accept_any,_make_int,_make_int)
  def getqobjij(self,i_,j_):
    """
    Obtains one coefficient from the quadratic term of the objective
  
    getqobjij(self,i_,j_)
      i: int. Row index of the coefficient.
      j: int. Column index of coefficient.
    returns: qoij
      qoij: double. The required coefficient.
    """
    qoij_ = ctypes.c_double()
    qoij_ = ctypes.c_double()
    res = __library__.MSK_XX_getqobjij(self.__nativep,i_,j_,ctypes.byref(qoij_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _qoij_return_value = qoij_.value
    return (_qoij_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_any,_accept_any,_accept_any,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector)
  def getsolution(self,whichsol_,skc_,skx_,skn_,xc_,xx_,y_,slc_,suc_,slx_,sux_,snx_):
    """
    Obtains the complete solution.
  
    getsolution(self,whichsol_,skc_,skx_,skn_,xc_,xx_,y_,slc_,suc_,slx_,sux_,snx_)
      whichsol: mosek.soltype. Selects a solution.
      skc: array of mosek.stakey. Status keys for the constraints.
      skx: array of mosek.stakey. Status keys for the variables.
      skn: array of mosek.stakey. Status keys for the conic constraints.
      xc: array of double. Primal constraint solution.
      xx: array of double. Primal variable solution.
      y: array of double. Vector of dual variables corresponding to the constraints.
      slc: array of double. Dual variables corresponding to the lower bounds on the constraints.
      suc: array of double. Dual variables corresponding to the upper bounds on the constraints.
      slx: array of double. Dual variables corresponding to the lower bounds on the variables.
      sux: array of double. Dual variables corresponding to the upper bounds on the variables.
      snx: array of double. Dual variables corresponding to the conic constraints on the variables.
    returns: prosta,solsta
      prosta: mosek.prosta. Problem status.
      solsta: mosek.solsta. Solution status.
    """
    prosta_ = ctypes.c_int32()
    solsta_ = ctypes.c_int32()
    if skc_ is not None and len(skc_) < self.getnumcon():
      raise ValueError("Array argument skc is not long enough")
    if isinstance(skc_,numpy.ndarray) and not skc_.flags.writeable:
      raise ValueError("Argument skc must be writable")
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))()
    else:
        _skc_tmp = None
    if skx_ is not None and len(skx_) < self.getnumvar():
      raise ValueError("Array argument skx is not long enough")
    if isinstance(skx_,numpy.ndarray) and not skx_.flags.writeable:
      raise ValueError("Argument skx must be writable")
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))()
    else:
        _skx_tmp = None
    if skn_ is not None and len(skn_) < self.getnumcone():
      raise ValueError("Array argument skn is not long enough")
    if isinstance(skn_,numpy.ndarray) and not skn_.flags.writeable:
      raise ValueError("Argument skn must be writable")
    if skn_ is not None:
        _skn_tmp = (ctypes.c_int32 * len(skn_))()
    else:
        _skn_tmp = None
    if xc_ is not None and len(xc_) < self.getnumcon():
      raise ValueError("Array argument xc is not long enough")
    if isinstance(xc_,numpy.ndarray) and not xc_.flags.writeable:
      raise ValueError("Argument xc must be writable")
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    if xx_ is not None and len(xx_) < self.getnumvar():
      raise ValueError("Array argument xx is not long enough")
    if isinstance(xx_,numpy.ndarray) and not xx_.flags.writeable:
      raise ValueError("Argument xx must be writable")
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    if y_ is not None and len(y_) < self.getnumcon():
      raise ValueError("Array argument y is not long enough")
    if isinstance(y_,numpy.ndarray) and not y_.flags.writeable:
      raise ValueError("Argument y must be writable")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    if slc_ is not None and len(slc_) < self.getnumcon():
      raise ValueError("Array argument slc is not long enough")
    if isinstance(slc_,numpy.ndarray) and not slc_.flags.writeable:
      raise ValueError("Argument slc must be writable")
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    if suc_ is not None and len(suc_) < self.getnumcon():
      raise ValueError("Array argument suc is not long enough")
    if isinstance(suc_,numpy.ndarray) and not suc_.flags.writeable:
      raise ValueError("Argument suc must be writable")
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    if slx_ is not None and len(slx_) < self.getnumvar():
      raise ValueError("Array argument slx is not long enough")
    if isinstance(slx_,numpy.ndarray) and not slx_.flags.writeable:
      raise ValueError("Argument slx must be writable")
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    if sux_ is not None and len(sux_) < self.getnumvar():
      raise ValueError("Array argument sux is not long enough")
    if isinstance(sux_,numpy.ndarray) and not sux_.flags.writeable:
      raise ValueError("Argument sux must be writable")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    if snx_ is not None and len(snx_) < self.getnumcone():
      raise ValueError("Array argument snx is not long enough")
    if isinstance(snx_,numpy.ndarray) and not snx_.flags.writeable:
      raise ValueError("Argument snx must be writable")
    if isinstance(snx_, numpy.ndarray) and snx_.dtype is numpy.dtype(numpy.float64) and snx_.flags.contiguous:
      _snx_copyarray = False
      _snx_tmp = ctypes.cast(snx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(snx_,array.ndarray) and snx_.dtype is numpy.dtype(array.float64) and  snx_.getsteplength() == 1:
      _snx_copyarray = False
      _snx_tmp = snx_.getdatacptr()
    elif snx_ is None:
      _snx_copyarray = False
      _snx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _snx_copyarray = True
      _snx_tmp = (ctypes.c_double * len(snx_))(*snx_)
    res = __library__.MSK_XX_getsolution(self.__nativep,whichsol_,ctypes.byref(prosta_),ctypes.byref(solsta_),_skc_tmp,_skx_tmp,_skn_tmp,_xc_tmp,_xx_tmp,_y_tmp,_slc_tmp,_suc_tmp,_slx_tmp,_sux_tmp,_snx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _prosta_return_value = prosta(prosta_.value)
    _solsta_return_value = solsta(solsta_.value)
    if skc_ is not None: skc_[:] = [ stakey(v) for v in _skc_tmp[0:len(skc_)] ]
    if skx_ is not None: skx_[:] = [ stakey(v) for v in _skx_tmp[0:len(skx_)] ]
    if skn_ is not None: skn_[:] = [ stakey(v) for v in _skn_tmp[0:len(skn_)] ]
    if _xc_copyarray:
      if isinstance(xc_, numpy.ndarray) and False:
        xc_[:] = list(_xc_tmp)
      else:
        xc_[:] = _xc_tmp
    if _xx_copyarray:
      if isinstance(xx_, numpy.ndarray) and False:
        xx_[:] = list(_xx_tmp)
      else:
        xx_[:] = _xx_tmp
    if _y_copyarray:
      if isinstance(y_, numpy.ndarray) and False:
        y_[:] = list(_y_tmp)
      else:
        y_[:] = _y_tmp
    if _slc_copyarray:
      if isinstance(slc_, numpy.ndarray) and False:
        slc_[:] = list(_slc_tmp)
      else:
        slc_[:] = _slc_tmp
    if _suc_copyarray:
      if isinstance(suc_, numpy.ndarray) and False:
        suc_[:] = list(_suc_tmp)
      else:
        suc_[:] = _suc_tmp
    if _slx_copyarray:
      if isinstance(slx_, numpy.ndarray) and False:
        slx_[:] = list(_slx_tmp)
      else:
        slx_[:] = _slx_tmp
    if _sux_copyarray:
      if isinstance(sux_, numpy.ndarray) and False:
        sux_[:] = list(_sux_tmp)
      else:
        sux_[:] = _sux_tmp
    if _snx_copyarray:
      if isinstance(snx_, numpy.ndarray) and False:
        snx_[:] = list(_snx_tmp)
      else:
        snx_[:] = _snx_tmp
    return (_prosta_return_value,_solsta_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_anyenum(accmode),_make_intvector,_accept_doublevector,_make_int)
  def getpbi(self,whichsol_,accmode_,sub_,pbi_,normalize_):
    """
    Deprecated.
  
    getpbi(self,whichsol_,accmode_,sub_,pbi_,normalize_)
      whichsol: mosek.soltype. Selects a solution.
      accmode: mosek.accmode. Defines whether bound infeasibilities related to constraints or variable are retrieved.
      sub: array of int. An array of constraint or variable indexes.
      pbi: array of double. Bound infeasibility.
      normalize: int. If non-zero, normalize with largest absolute value of input data.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if pbi_ is not None and len(pbi_) < (len_):
      raise ValueError("Array argument pbi is not long enough")
    if isinstance(pbi_,numpy.ndarray) and not pbi_.flags.writeable:
      raise ValueError("Argument pbi must be writable")
    if isinstance(pbi_, numpy.ndarray) and pbi_.dtype is numpy.dtype(numpy.float64) and pbi_.flags.contiguous:
      _pbi_copyarray = False
      _pbi_tmp = ctypes.cast(pbi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(pbi_,array.ndarray) and pbi_.dtype is numpy.dtype(array.float64) and  pbi_.getsteplength() == 1:
      _pbi_copyarray = False
      _pbi_tmp = pbi_.getdatacptr()
    elif pbi_ is None:
      _pbi_copyarray = False
      _pbi_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _pbi_copyarray = True
      _pbi_tmp = (ctypes.c_double * len(pbi_))(*pbi_)
    res = __library__.MSK_XX_getpbi(self.__nativep,whichsol_,accmode_,_sub_tmp,len_,_pbi_tmp,normalize_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _pbi_copyarray:
      if isinstance(pbi_, numpy.ndarray) and False:
        pbi_[:] = list(_pbi_tmp)
      else:
        pbi_[:] = _pbi_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_anyenum(accmode),_make_intvector,_accept_doublevector)
  def getdbi(self,whichsol_,accmode_,sub_,dbi_):
    """
    Deprecated.
  
    getdbi(self,whichsol_,accmode_,sub_,dbi_)
      whichsol: mosek.soltype. Selects a solution.
      accmode: mosek.accmode. Defines whether sub contains constraint or variable indexes.
      sub: array of int. Indexes of constraints or variables.
      dbi: array of double. Dual bound infeasibility.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if dbi_ is not None and len(dbi_) < (len_):
      raise ValueError("Array argument dbi is not long enough")
    if isinstance(dbi_,numpy.ndarray) and not dbi_.flags.writeable:
      raise ValueError("Argument dbi must be writable")
    if isinstance(dbi_, numpy.ndarray) and dbi_.dtype is numpy.dtype(numpy.float64) and dbi_.flags.contiguous:
      _dbi_copyarray = False
      _dbi_tmp = ctypes.cast(dbi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(dbi_,array.ndarray) and dbi_.dtype is numpy.dtype(array.float64) and  dbi_.getsteplength() == 1:
      _dbi_copyarray = False
      _dbi_tmp = dbi_.getdatacptr()
    elif dbi_ is None:
      _dbi_copyarray = False
      _dbi_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _dbi_copyarray = True
      _dbi_tmp = (ctypes.c_double * len(dbi_))(*dbi_)
    res = __library__.MSK_XX_getdbi(self.__nativep,whichsol_,accmode_,_sub_tmp,len_,_dbi_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _dbi_copyarray:
      if isinstance(dbi_, numpy.ndarray) and False:
        dbi_[:] = list(_dbi_tmp)
      else:
        dbi_[:] = _dbi_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_anyenum(accmode),_make_intvector,_accept_doublevector,_make_int)
  def getdeqi(self,whichsol_,accmode_,sub_,deqi_,normalize_):
    """
    Deprecated.
  
    getdeqi(self,whichsol_,accmode_,sub_,deqi_,normalize_)
      whichsol: mosek.soltype. Selects a solution.
      accmode: mosek.accmode. Defines whether equation infeasibilities for constraints or for variables are retrieved.
      sub: array of int. Indexes of constraints or variables.
      deqi: array of double. Dual equation infeasibilities corresponding to constraints or variables.
      normalize: int. If non-zero, normalize with largest absolute value of input data.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if deqi_ is not None and len(deqi_) < (len_):
      raise ValueError("Array argument deqi is not long enough")
    if isinstance(deqi_,numpy.ndarray) and not deqi_.flags.writeable:
      raise ValueError("Argument deqi must be writable")
    if isinstance(deqi_, numpy.ndarray) and deqi_.dtype is numpy.dtype(numpy.float64) and deqi_.flags.contiguous:
      _deqi_copyarray = False
      _deqi_tmp = ctypes.cast(deqi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(deqi_,array.ndarray) and deqi_.dtype is numpy.dtype(array.float64) and  deqi_.getsteplength() == 1:
      _deqi_copyarray = False
      _deqi_tmp = deqi_.getdatacptr()
    elif deqi_ is None:
      _deqi_copyarray = False
      _deqi_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _deqi_copyarray = True
      _deqi_tmp = (ctypes.c_double * len(deqi_))(*deqi_)
    res = __library__.MSK_XX_getdeqi(self.__nativep,whichsol_,accmode_,_sub_tmp,len_,_deqi_tmp,normalize_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _deqi_copyarray:
      if isinstance(deqi_, numpy.ndarray) and False:
        deqi_[:] = list(_deqi_tmp)
      else:
        deqi_[:] = _deqi_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector,_make_int)
  def getpeqi(self,whichsol_,sub_,peqi_,normalize_):
    """
    Deprecated.
  
    getpeqi(self,whichsol_,sub_,peqi_,normalize_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. Constraint indexes for which to calculate the equation infeasibility.
      peqi: array of double. Equation infeasibility.
      normalize: int. If non-zero, normalize with largest absolute value of input data.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if peqi_ is not None and len(peqi_) < (len_):
      raise ValueError("Array argument peqi is not long enough")
    if isinstance(peqi_,numpy.ndarray) and not peqi_.flags.writeable:
      raise ValueError("Argument peqi must be writable")
    if isinstance(peqi_, numpy.ndarray) and peqi_.dtype is numpy.dtype(numpy.float64) and peqi_.flags.contiguous:
      _peqi_copyarray = False
      _peqi_tmp = ctypes.cast(peqi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(peqi_,array.ndarray) and peqi_.dtype is numpy.dtype(array.float64) and  peqi_.getsteplength() == 1:
      _peqi_copyarray = False
      _peqi_tmp = peqi_.getdatacptr()
    elif peqi_ is None:
      _peqi_copyarray = False
      _peqi_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _peqi_copyarray = True
      _peqi_tmp = (ctypes.c_double * len(peqi_))(*peqi_)
    res = __library__.MSK_XX_getpeqi(self.__nativep,whichsol_,_sub_tmp,len_,_peqi_tmp,normalize_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _peqi_copyarray:
      if isinstance(peqi_, numpy.ndarray) and False:
        peqi_[:] = list(_peqi_tmp)
      else:
        peqi_[:] = _peqi_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getinti(self,whichsol_,sub_,inti_):
    """
    Deprecated.
  
    getinti(self,whichsol_,sub_,inti_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. Variable indexes for which to calculate the integer infeasibility.
      inti: array of double. Integer infeasibility.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if inti_ is not None and len(inti_) < (len_):
      raise ValueError("Array argument inti is not long enough")
    if isinstance(inti_,numpy.ndarray) and not inti_.flags.writeable:
      raise ValueError("Argument inti must be writable")
    if isinstance(inti_, numpy.ndarray) and inti_.dtype is numpy.dtype(numpy.float64) and inti_.flags.contiguous:
      _inti_copyarray = False
      _inti_tmp = ctypes.cast(inti_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(inti_,array.ndarray) and inti_.dtype is numpy.dtype(array.float64) and  inti_.getsteplength() == 1:
      _inti_copyarray = False
      _inti_tmp = inti_.getdatacptr()
    elif inti_ is None:
      _inti_copyarray = False
      _inti_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _inti_copyarray = True
      _inti_tmp = (ctypes.c_double * len(inti_))(*inti_)
    res = __library__.MSK_XX_getinti(self.__nativep,whichsol_,_sub_tmp,len_,_inti_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _inti_copyarray:
      if isinstance(inti_, numpy.ndarray) and False:
        inti_[:] = list(_inti_tmp)
      else:
        inti_[:] = _inti_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getpcni(self,whichsol_,sub_,pcni_):
    """
    Deprecated.
  
    getpcni(self,whichsol_,sub_,pcni_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. Constraint indexes for which to calculate the equation infeasibility.
      pcni: array of double. Primal cone infeasibility.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if pcni_ is not None and len(pcni_) < (len_):
      raise ValueError("Array argument pcni is not long enough")
    if isinstance(pcni_,numpy.ndarray) and not pcni_.flags.writeable:
      raise ValueError("Argument pcni must be writable")
    if isinstance(pcni_, numpy.ndarray) and pcni_.dtype is numpy.dtype(numpy.float64) and pcni_.flags.contiguous:
      _pcni_copyarray = False
      _pcni_tmp = ctypes.cast(pcni_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(pcni_,array.ndarray) and pcni_.dtype is numpy.dtype(array.float64) and  pcni_.getsteplength() == 1:
      _pcni_copyarray = False
      _pcni_tmp = pcni_.getdatacptr()
    elif pcni_ is None:
      _pcni_copyarray = False
      _pcni_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _pcni_copyarray = True
      _pcni_tmp = (ctypes.c_double * len(pcni_))(*pcni_)
    res = __library__.MSK_XX_getpcni(self.__nativep,whichsol_,_sub_tmp,len_,_pcni_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _pcni_copyarray:
      if isinstance(pcni_, numpy.ndarray) and False:
        pcni_[:] = list(_pcni_tmp)
      else:
        pcni_[:] = _pcni_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getdcni(self,whichsol_,sub_,dcni_):
    """
    Deprecated.
  
    getdcni(self,whichsol_,sub_,dcni_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. Constraint indexes to calculate equation infeasibility for.
      dcni: array of double. Dual cone infeasibility.
    """
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    len_ = None
    if len_ is None:
      len_ = len(sub_)
    else:
      len_ = min(len_,len(sub_))
    if dcni_ is not None and len(dcni_) < (len_):
      raise ValueError("Array argument dcni is not long enough")
    if isinstance(dcni_,numpy.ndarray) and not dcni_.flags.writeable:
      raise ValueError("Argument dcni must be writable")
    if isinstance(dcni_, numpy.ndarray) and dcni_.dtype is numpy.dtype(numpy.float64) and dcni_.flags.contiguous:
      _dcni_copyarray = False
      _dcni_tmp = ctypes.cast(dcni_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(dcni_,array.ndarray) and dcni_.dtype is numpy.dtype(array.float64) and  dcni_.getsteplength() == 1:
      _dcni_copyarray = False
      _dcni_tmp = dcni_.getdatacptr()
    elif dcni_ is None:
      _dcni_copyarray = False
      _dcni_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _dcni_copyarray = True
      _dcni_tmp = (ctypes.c_double * len(dcni_))(*dcni_)
    res = __library__.MSK_XX_getdcni(self.__nativep,whichsol_,_sub_tmp,len_,_dcni_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _dcni_copyarray:
      if isinstance(dcni_, numpy.ndarray) and False:
        dcni_[:] = list(_dcni_tmp)
      else:
        dcni_[:] = _dcni_tmp
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_accept_anyenum(soltype))
  def getsolutioni(self,accmode_,i_,whichsol_):
    """
    Obtains the solution for a single constraint or variable.
  
    getsolutioni(self,accmode_,i_,whichsol_)
      accmode: mosek.accmode. Defines whether solution information for a constraint or for a variable is retrieved.
      i: int. Index of the constraint or variable.
      whichsol: mosek.soltype. Selects a solution.
    returns: sk,x,sl,su,sn
      sk: mosek.stakey. Status key of the constraint of variable.
      x: double. Solution value of the primal variable.
      sl: double. Solution value of the dual variable associated with the lower bound.
      su: double. Solution value of the dual variable associated with the upper bound.
      sn: double. Solution value of the dual variable associated with the cone constraint.
    """
    sk_ = ctypes.c_int32()
    x_ = ctypes.c_double()
    x_ = ctypes.c_double()
    sl_ = ctypes.c_double()
    sl_ = ctypes.c_double()
    su_ = ctypes.c_double()
    su_ = ctypes.c_double()
    sn_ = ctypes.c_double()
    sn_ = ctypes.c_double()
    res = __library__.MSK_XX_getsolutioni(self.__nativep,accmode_,i_,whichsol_,ctypes.byref(sk_),ctypes.byref(x_),ctypes.byref(sl_),ctypes.byref(su_),ctypes.byref(sn_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _sk_return_value = stakey(sk_.value)
    _x_return_value = x_.value
    _sl_return_value = sl_.value
    _su_return_value = su_.value
    _sn_return_value = sn_.value
    return (_sk_return_value,_x_return_value,_sl_return_value,_su_return_value,_sn_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getsolsta(self,whichsol_):
    """
    Obtains the solution status.
  
    getsolsta(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: solsta
      solsta: mosek.solsta. Solution status.
    """
    solsta_ = ctypes.c_int32()
    res = __library__.MSK_XX_getsolsta(self.__nativep,whichsol_,ctypes.byref(solsta_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _solsta_return_value = solsta(solsta_.value)
    return (_solsta_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getprosta(self,whichsol_):
    """
    Obtains the problem status.
  
    getprosta(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: prosta
      prosta: mosek.prosta. Problem status.
    """
    prosta_ = ctypes.c_int32()
    res = __library__.MSK_XX_getprosta(self.__nativep,whichsol_,ctypes.byref(prosta_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _prosta_return_value = prosta(prosta_.value)
    return (_prosta_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_any)
  def getskc(self,whichsol_,skc_):
    """
    Obtains the status keys for the constraints.
  
    getskc(self,whichsol_,skc_)
      whichsol: mosek.soltype. Selects a solution.
      skc: array of mosek.stakey. Status keys for the constraints.
    """
    if skc_ is not None and len(skc_) < self.getnumcon():
      raise ValueError("Array argument skc is not long enough")
    if isinstance(skc_,numpy.ndarray) and not skc_.flags.writeable:
      raise ValueError("Argument skc must be writable")
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))()
    else:
        _skc_tmp = None
    res = __library__.MSK_XX_getskc(self.__nativep,whichsol_,_skc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if skc_ is not None: skc_[:] = [ stakey(v) for v in _skc_tmp[0:len(skc_)] ]
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_any)
  def getskx(self,whichsol_,skx_):
    """
    Obtains the status keys for the scalar variables.
  
    getskx(self,whichsol_,skx_)
      whichsol: mosek.soltype. Selects a solution.
      skx: array of mosek.stakey. Status keys for the variables.
    """
    if skx_ is not None and len(skx_) < self.getnumvar():
      raise ValueError("Array argument skx is not long enough")
    if isinstance(skx_,numpy.ndarray) and not skx_.flags.writeable:
      raise ValueError("Argument skx must be writable")
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))()
    else:
        _skx_tmp = None
    res = __library__.MSK_XX_getskx(self.__nativep,whichsol_,_skx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if skx_ is not None: skx_[:] = [ stakey(v) for v in _skx_tmp[0:len(skx_)] ]
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getxc(self,whichsol_,xc_):
    """
    Obtains the xc vector for a solution.
  
    getxc(self,whichsol_,xc_)
      whichsol: mosek.soltype. Selects a solution.
      xc: array of double. The xc vector.
    """
    if xc_ is not None and len(xc_) < self.getnumcon():
      raise ValueError("Array argument xc is not long enough")
    if isinstance(xc_,numpy.ndarray) and not xc_.flags.writeable:
      raise ValueError("Argument xc must be writable")
    if xc_ is None:
      raise ValueError("Argument xc may not be None")
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    res = __library__.MSK_XX_getxc(self.__nativep,whichsol_,_xc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _xc_copyarray:
      if isinstance(xc_, numpy.ndarray) and False:
        xc_[:] = list(_xc_tmp)
      else:
        xc_[:] = _xc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getxx(self,whichsol_,xx_):
    """
    Obtains the xx vector for a solution.
  
    getxx(self,whichsol_,xx_)
      whichsol: mosek.soltype. Selects a solution.
      xx: array of double. The xx vector.
    """
    if xx_ is not None and len(xx_) < self.getnumvar():
      raise ValueError("Array argument xx is not long enough")
    if isinstance(xx_,numpy.ndarray) and not xx_.flags.writeable:
      raise ValueError("Argument xx must be writable")
    if xx_ is None:
      raise ValueError("Argument xx may not be None")
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    res = __library__.MSK_XX_getxx(self.__nativep,whichsol_,_xx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _xx_copyarray:
      if isinstance(xx_, numpy.ndarray) and False:
        xx_[:] = list(_xx_tmp)
      else:
        xx_[:] = _xx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def gety(self,whichsol_,y_):
    """
    Obtains the y vector for a solution.
  
    gety(self,whichsol_,y_)
      whichsol: mosek.soltype. Selects a solution.
      y: array of double. The y vector.
    """
    if y_ is not None and len(y_) < self.getnumcon():
      raise ValueError("Array argument y is not long enough")
    if isinstance(y_,numpy.ndarray) and not y_.flags.writeable:
      raise ValueError("Argument y must be writable")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_gety(self.__nativep,whichsol_,_y_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _y_copyarray:
      if isinstance(y_, numpy.ndarray) and False:
        y_[:] = list(_y_tmp)
      else:
        y_[:] = _y_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getslc(self,whichsol_,slc_):
    """
    Obtains the slc vector for a solution.
  
    getslc(self,whichsol_,slc_)
      whichsol: mosek.soltype. Selects a solution.
      slc: array of double. The slc vector.
    """
    if slc_ is not None and len(slc_) < self.getnumcon():
      raise ValueError("Array argument slc is not long enough")
    if isinstance(slc_,numpy.ndarray) and not slc_.flags.writeable:
      raise ValueError("Argument slc must be writable")
    if slc_ is None:
      raise ValueError("Argument slc may not be None")
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    res = __library__.MSK_XX_getslc(self.__nativep,whichsol_,_slc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _slc_copyarray:
      if isinstance(slc_, numpy.ndarray) and False:
        slc_[:] = list(_slc_tmp)
      else:
        slc_[:] = _slc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getsuc(self,whichsol_,suc_):
    """
    Obtains the suc vector for a solution.
  
    getsuc(self,whichsol_,suc_)
      whichsol: mosek.soltype. Selects a solution.
      suc: array of double. The suc vector.
    """
    if suc_ is not None and len(suc_) < self.getnumcon():
      raise ValueError("Array argument suc is not long enough")
    if isinstance(suc_,numpy.ndarray) and not suc_.flags.writeable:
      raise ValueError("Argument suc must be writable")
    if suc_ is None:
      raise ValueError("Argument suc may not be None")
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    res = __library__.MSK_XX_getsuc(self.__nativep,whichsol_,_suc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _suc_copyarray:
      if isinstance(suc_, numpy.ndarray) and False:
        suc_[:] = list(_suc_tmp)
      else:
        suc_[:] = _suc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getslx(self,whichsol_,slx_):
    """
    Obtains the slx vector for a solution.
  
    getslx(self,whichsol_,slx_)
      whichsol: mosek.soltype. Selects a solution.
      slx: array of double. The slx vector.
    """
    if slx_ is not None and len(slx_) < self.getnumvar():
      raise ValueError("Array argument slx is not long enough")
    if isinstance(slx_,numpy.ndarray) and not slx_.flags.writeable:
      raise ValueError("Argument slx must be writable")
    if slx_ is None:
      raise ValueError("Argument slx may not be None")
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    res = __library__.MSK_XX_getslx(self.__nativep,whichsol_,_slx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _slx_copyarray:
      if isinstance(slx_, numpy.ndarray) and False:
        slx_[:] = list(_slx_tmp)
      else:
        slx_[:] = _slx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getsux(self,whichsol_,sux_):
    """
    Obtains the sux vector for a solution.
  
    getsux(self,whichsol_,sux_)
      whichsol: mosek.soltype. Selects a solution.
      sux: array of double. The sux vector.
    """
    if sux_ is not None and len(sux_) < self.getnumvar():
      raise ValueError("Array argument sux is not long enough")
    if isinstance(sux_,numpy.ndarray) and not sux_.flags.writeable:
      raise ValueError("Argument sux must be writable")
    if sux_ is None:
      raise ValueError("Argument sux may not be None")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    res = __library__.MSK_XX_getsux(self.__nativep,whichsol_,_sux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _sux_copyarray:
      if isinstance(sux_, numpy.ndarray) and False:
        sux_[:] = list(_sux_tmp)
      else:
        sux_[:] = _sux_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def getsnx(self,whichsol_,snx_):
    """
    Obtains the snx vector for a solution.
  
    getsnx(self,whichsol_,snx_)
      whichsol: mosek.soltype. Selects a solution.
      snx: array of double. The snx vector.
    """
    if snx_ is not None and len(snx_) < self.getnumvar():
      raise ValueError("Array argument snx is not long enough")
    if isinstance(snx_,numpy.ndarray) and not snx_.flags.writeable:
      raise ValueError("Argument snx must be writable")
    if snx_ is None:
      raise ValueError("Argument snx may not be None")
    if isinstance(snx_, numpy.ndarray) and snx_.dtype is numpy.dtype(numpy.float64) and snx_.flags.contiguous:
      _snx_copyarray = False
      _snx_tmp = ctypes.cast(snx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(snx_,array.ndarray) and snx_.dtype is numpy.dtype(array.float64) and  snx_.getsteplength() == 1:
      _snx_copyarray = False
      _snx_tmp = snx_.getdatacptr()
    elif snx_ is None:
      _snx_copyarray = False
      _snx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _snx_copyarray = True
      _snx_tmp = (ctypes.c_double * len(snx_))(*snx_)
    res = __library__.MSK_XX_getsnx(self.__nativep,whichsol_,_snx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _snx_copyarray:
      if isinstance(snx_, numpy.ndarray) and False:
        snx_[:] = list(_snx_tmp)
      else:
        snx_[:] = _snx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_any)
  def getskcslice(self,whichsol_,first_,last_,skc_):
    """
    Obtains the status keys for the constraints.
  
    getskcslice(self,whichsol_,first_,last_,skc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      skc: array of mosek.stakey. Status keys for the constraints.
    """
    if skc_ is not None and len(skc_) < ((last_) - (first_)):
      raise ValueError("Array argument skc is not long enough")
    if isinstance(skc_,numpy.ndarray) and not skc_.flags.writeable:
      raise ValueError("Argument skc must be writable")
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))()
    else:
        _skc_tmp = None
    res = __library__.MSK_XX_getskcslice(self.__nativep,whichsol_,first_,last_,_skc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if skc_ is not None: skc_[:] = [ stakey(v) for v in _skc_tmp[0:len(skc_)] ]
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_any)
  def getskxslice(self,whichsol_,first_,last_,skx_):
    """
    Obtains the status keys for the variables.
  
    getskxslice(self,whichsol_,first_,last_,skx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      skx: array of mosek.stakey. Status keys for the variables.
    """
    if skx_ is not None and len(skx_) < ((last_) - (first_)):
      raise ValueError("Array argument skx is not long enough")
    if isinstance(skx_,numpy.ndarray) and not skx_.flags.writeable:
      raise ValueError("Argument skx must be writable")
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))()
    else:
        _skx_tmp = None
    res = __library__.MSK_XX_getskxslice(self.__nativep,whichsol_,first_,last_,_skx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if skx_ is not None: skx_[:] = [ stakey(v) for v in _skx_tmp[0:len(skx_)] ]
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getxcslice(self,whichsol_,first_,last_,xc_):
    """
    Obtains a slice of the xc vector for a solution.
  
    getxcslice(self,whichsol_,first_,last_,xc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      xc: array of double. Primal constraint solution.
    """
    if xc_ is not None and len(xc_) < ((last_) - (first_)):
      raise ValueError("Array argument xc is not long enough")
    if isinstance(xc_,numpy.ndarray) and not xc_.flags.writeable:
      raise ValueError("Argument xc must be writable")
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    res = __library__.MSK_XX_getxcslice(self.__nativep,whichsol_,first_,last_,_xc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _xc_copyarray:
      if isinstance(xc_, numpy.ndarray) and False:
        xc_[:] = list(_xc_tmp)
      else:
        xc_[:] = _xc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getxxslice(self,whichsol_,first_,last_,xx_):
    """
    Obtains a slice of the xx vector for a solution.
  
    getxxslice(self,whichsol_,first_,last_,xx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      xx: array of double. Primal variable solution.
    """
    if xx_ is not None and len(xx_) < ((last_) - (first_)):
      raise ValueError("Array argument xx is not long enough")
    if isinstance(xx_,numpy.ndarray) and not xx_.flags.writeable:
      raise ValueError("Argument xx must be writable")
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    res = __library__.MSK_XX_getxxslice(self.__nativep,whichsol_,first_,last_,_xx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _xx_copyarray:
      if isinstance(xx_, numpy.ndarray) and False:
        xx_[:] = list(_xx_tmp)
      else:
        xx_[:] = _xx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getyslice(self,whichsol_,first_,last_,y_):
    """
    Obtains a slice of the y vector for a solution.
  
    getyslice(self,whichsol_,first_,last_,y_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      y: array of double. Vector of dual variables corresponding to the constraints.
    """
    if y_ is not None and len(y_) < ((last_) - (first_)):
      raise ValueError("Array argument y is not long enough")
    if isinstance(y_,numpy.ndarray) and not y_.flags.writeable:
      raise ValueError("Argument y must be writable")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_getyslice(self.__nativep,whichsol_,first_,last_,_y_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _y_copyarray:
      if isinstance(y_, numpy.ndarray) and False:
        y_[:] = list(_y_tmp)
      else:
        y_[:] = _y_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getslcslice(self,whichsol_,first_,last_,slc_):
    """
    Obtains a slice of the slc vector for a solution.
  
    getslcslice(self,whichsol_,first_,last_,slc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      slc: array of double. Dual variables corresponding to the lower bounds on the constraints.
    """
    if slc_ is not None and len(slc_) < ((last_) - (first_)):
      raise ValueError("Array argument slc is not long enough")
    if isinstance(slc_,numpy.ndarray) and not slc_.flags.writeable:
      raise ValueError("Argument slc must be writable")
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    res = __library__.MSK_XX_getslcslice(self.__nativep,whichsol_,first_,last_,_slc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _slc_copyarray:
      if isinstance(slc_, numpy.ndarray) and False:
        slc_[:] = list(_slc_tmp)
      else:
        slc_[:] = _slc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getsucslice(self,whichsol_,first_,last_,suc_):
    """
    Obtains a slice of the suc vector for a solution.
  
    getsucslice(self,whichsol_,first_,last_,suc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      suc: array of double. Dual variables corresponding to the upper bounds on the constraints.
    """
    if suc_ is not None and len(suc_) < ((last_) - (first_)):
      raise ValueError("Array argument suc is not long enough")
    if isinstance(suc_,numpy.ndarray) and not suc_.flags.writeable:
      raise ValueError("Argument suc must be writable")
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    res = __library__.MSK_XX_getsucslice(self.__nativep,whichsol_,first_,last_,_suc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _suc_copyarray:
      if isinstance(suc_, numpy.ndarray) and False:
        suc_[:] = list(_suc_tmp)
      else:
        suc_[:] = _suc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getslxslice(self,whichsol_,first_,last_,slx_):
    """
    Obtains a slice of the slx vector for a solution.
  
    getslxslice(self,whichsol_,first_,last_,slx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      slx: array of double. Dual variables corresponding to the lower bounds on the variables.
    """
    if slx_ is not None and len(slx_) < ((last_) - (first_)):
      raise ValueError("Array argument slx is not long enough")
    if isinstance(slx_,numpy.ndarray) and not slx_.flags.writeable:
      raise ValueError("Argument slx must be writable")
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    res = __library__.MSK_XX_getslxslice(self.__nativep,whichsol_,first_,last_,_slx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _slx_copyarray:
      if isinstance(slx_, numpy.ndarray) and False:
        slx_[:] = list(_slx_tmp)
      else:
        slx_[:] = _slx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getsuxslice(self,whichsol_,first_,last_,sux_):
    """
    Obtains a slice of the sux vector for a solution.
  
    getsuxslice(self,whichsol_,first_,last_,sux_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      sux: array of double. Dual variables corresponding to the upper bounds on the variables.
    """
    if sux_ is not None and len(sux_) < ((last_) - (first_)):
      raise ValueError("Array argument sux is not long enough")
    if isinstance(sux_,numpy.ndarray) and not sux_.flags.writeable:
      raise ValueError("Argument sux must be writable")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    res = __library__.MSK_XX_getsuxslice(self.__nativep,whichsol_,first_,last_,_sux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _sux_copyarray:
      if isinstance(sux_, numpy.ndarray) and False:
        sux_[:] = list(_sux_tmp)
      else:
        sux_[:] = _sux_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getsnxslice(self,whichsol_,first_,last_,snx_):
    """
    Obtains a slice of the snx vector for a solution.
  
    getsnxslice(self,whichsol_,first_,last_,snx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      snx: array of double. Dual variables corresponding to the conic constraints on the variables.
    """
    if snx_ is not None and len(snx_) < ((last_) - (first_)):
      raise ValueError("Array argument snx is not long enough")
    if isinstance(snx_,numpy.ndarray) and not snx_.flags.writeable:
      raise ValueError("Argument snx must be writable")
    if isinstance(snx_, numpy.ndarray) and snx_.dtype is numpy.dtype(numpy.float64) and snx_.flags.contiguous:
      _snx_copyarray = False
      _snx_tmp = ctypes.cast(snx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(snx_,array.ndarray) and snx_.dtype is numpy.dtype(array.float64) and  snx_.getsteplength() == 1:
      _snx_copyarray = False
      _snx_tmp = snx_.getdatacptr()
    elif snx_ is None:
      _snx_copyarray = False
      _snx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _snx_copyarray = True
      _snx_tmp = (ctypes.c_double * len(snx_))(*snx_)
    res = __library__.MSK_XX_getsnxslice(self.__nativep,whichsol_,first_,last_,_snx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _snx_copyarray:
      if isinstance(snx_, numpy.ndarray) and False:
        snx_[:] = list(_snx_tmp)
      else:
        snx_[:] = _snx_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_accept_doublevector)
  def getbarxj(self,whichsol_,j_,barxj_):
    """
    Obtains the primal solution for a semidefinite variable.
  
    getbarxj(self,whichsol_,j_,barxj_)
      whichsol: mosek.soltype. Selects a solution.
      j: int. Index of the semidefinite variable.
      barxj: array of double. Value of the j'th variable of barx.
    """
    if barxj_ is not None and len(barxj_) < self.getlenbarvarj((j_)):
      raise ValueError("Array argument barxj is not long enough")
    if isinstance(barxj_,numpy.ndarray) and not barxj_.flags.writeable:
      raise ValueError("Argument barxj must be writable")
    if barxj_ is None:
      raise ValueError("Argument barxj may not be None")
    if isinstance(barxj_, numpy.ndarray) and barxj_.dtype is numpy.dtype(numpy.float64) and barxj_.flags.contiguous:
      _barxj_copyarray = False
      _barxj_tmp = ctypes.cast(barxj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(barxj_,array.ndarray) and barxj_.dtype is numpy.dtype(array.float64) and  barxj_.getsteplength() == 1:
      _barxj_copyarray = False
      _barxj_tmp = barxj_.getdatacptr()
    elif barxj_ is None:
      _barxj_copyarray = False
      _barxj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _barxj_copyarray = True
      _barxj_tmp = (ctypes.c_double * len(barxj_))(*barxj_)
    res = __library__.MSK_XX_getbarxj(self.__nativep,whichsol_,j_,_barxj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _barxj_copyarray:
      if isinstance(barxj_, numpy.ndarray) and False:
        barxj_[:] = list(_barxj_tmp)
      else:
        barxj_[:] = _barxj_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_accept_doublevector)
  def getbarsj(self,whichsol_,j_,barsj_):
    """
    Obtains the dual solution for a semidefinite variable.
  
    getbarsj(self,whichsol_,j_,barsj_)
      whichsol: mosek.soltype. Selects a solution.
      j: int. Index of the semidefinite variable.
      barsj: array of double. Value of the j'th variable of barx.
    """
    if barsj_ is not None and len(barsj_) < self.getlenbarvarj((j_)):
      raise ValueError("Array argument barsj is not long enough")
    if isinstance(barsj_,numpy.ndarray) and not barsj_.flags.writeable:
      raise ValueError("Argument barsj must be writable")
    if barsj_ is None:
      raise ValueError("Argument barsj may not be None")
    if isinstance(barsj_, numpy.ndarray) and barsj_.dtype is numpy.dtype(numpy.float64) and barsj_.flags.contiguous:
      _barsj_copyarray = False
      _barsj_tmp = ctypes.cast(barsj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(barsj_,array.ndarray) and barsj_.dtype is numpy.dtype(array.float64) and  barsj_.getsteplength() == 1:
      _barsj_copyarray = False
      _barsj_tmp = barsj_.getdatacptr()
    elif barsj_ is None:
      _barsj_copyarray = False
      _barsj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _barsj_copyarray = True
      _barsj_tmp = (ctypes.c_double * len(barsj_))(*barsj_)
    res = __library__.MSK_XX_getbarsj(self.__nativep,whichsol_,j_,_barsj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _barsj_copyarray:
      if isinstance(barsj_, numpy.ndarray) and False:
        barsj_[:] = list(_barsj_tmp)
      else:
        barsj_[:] = _barsj_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_anyenumvector(stakey))
  def putskc(self,whichsol_,skc_):
    """
    Sets the status keys for the constraints.
  
    putskc(self,whichsol_,skc_)
      whichsol: mosek.soltype. Selects a solution.
      skc: array of mosek.stakey. Status keys for the constraints.
    """
    if skc_ is not None and len(skc_) < self.getnumcon():
      raise ValueError("Array argument skc is not long enough")
    if skc_ is None:
      raise ValueError("Argument skc cannot be None")
    if skc_ is None:
      raise ValueError("Argument skc may not be None")
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))(*skc_)
    else:
        _skc_tmp = None
    res = __library__.MSK_XX_putskc(self.__nativep,whichsol_,_skc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_anyenumvector(stakey))
  def putskx(self,whichsol_,skx_):
    """
    Sets the status keys for the scalar variables.
  
    putskx(self,whichsol_,skx_)
      whichsol: mosek.soltype. Selects a solution.
      skx: array of mosek.stakey. Status keys for the variables.
    """
    if skx_ is not None and len(skx_) < self.getnumvar():
      raise ValueError("Array argument skx is not long enough")
    if skx_ is None:
      raise ValueError("Argument skx cannot be None")
    if skx_ is None:
      raise ValueError("Argument skx may not be None")
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))(*skx_)
    else:
        _skx_tmp = None
    res = __library__.MSK_XX_putskx(self.__nativep,whichsol_,_skx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_doublevector)
  def putxc(self,whichsol_,xc_):
    """
    Sets the xc vector for a solution.
  
    putxc(self,whichsol_,xc_)
      whichsol: mosek.soltype. Selects a solution.
      xc: array of double. The xc vector.
    """
    if xc_ is not None and len(xc_) < self.getnumcon():
      raise ValueError("Array argument xc is not long enough")
    if isinstance(xc_,numpy.ndarray) and not xc_.flags.writeable:
      raise ValueError("Argument xc must be writable")
    if xc_ is None:
      raise ValueError("Argument xc may not be None")
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    res = __library__.MSK_XX_putxc(self.__nativep,whichsol_,_xc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _xc_copyarray:
      if isinstance(xc_, numpy.ndarray) and False:
        xc_[:] = list(_xc_tmp)
      else:
        xc_[:] = _xc_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putxx(self,whichsol_,xx_):
    """
    Sets the xx vector for a solution.
  
    putxx(self,whichsol_,xx_)
      whichsol: mosek.soltype. Selects a solution.
      xx: array of double. The xx vector.
    """
    if xx_ is not None and len(xx_) < self.getnumvar():
      raise ValueError("Array argument xx is not long enough")
    if xx_ is None:
      raise ValueError("Argument xx cannot be None")
    if xx_ is None:
      raise ValueError("Argument xx may not be None")
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    res = __library__.MSK_XX_putxx(self.__nativep,whichsol_,_xx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def puty(self,whichsol_,y_):
    """
    Sets the y vector for a solution.
  
    puty(self,whichsol_,y_)
      whichsol: mosek.soltype. Selects a solution.
      y: array of double. The y vector.
    """
    if y_ is not None and len(y_) < self.getnumcon():
      raise ValueError("Array argument y is not long enough")
    if y_ is None:
      raise ValueError("Argument y cannot be None")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_puty(self.__nativep,whichsol_,_y_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putslc(self,whichsol_,slc_):
    """
    Sets the slc vector for a solution.
  
    putslc(self,whichsol_,slc_)
      whichsol: mosek.soltype. Selects a solution.
      slc: array of double. The slc vector.
    """
    if slc_ is not None and len(slc_) < self.getnumcon():
      raise ValueError("Array argument slc is not long enough")
    if slc_ is None:
      raise ValueError("Argument slc cannot be None")
    if slc_ is None:
      raise ValueError("Argument slc may not be None")
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    res = __library__.MSK_XX_putslc(self.__nativep,whichsol_,_slc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putsuc(self,whichsol_,suc_):
    """
    Sets the suc vector for a solution.
  
    putsuc(self,whichsol_,suc_)
      whichsol: mosek.soltype. Selects a solution.
      suc: array of double. The suc vector.
    """
    if suc_ is not None and len(suc_) < self.getnumcon():
      raise ValueError("Array argument suc is not long enough")
    if suc_ is None:
      raise ValueError("Argument suc cannot be None")
    if suc_ is None:
      raise ValueError("Argument suc may not be None")
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    res = __library__.MSK_XX_putsuc(self.__nativep,whichsol_,_suc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putslx(self,whichsol_,slx_):
    """
    Sets the slx vector for a solution.
  
    putslx(self,whichsol_,slx_)
      whichsol: mosek.soltype. Selects a solution.
      slx: array of double. The slx vector.
    """
    if slx_ is not None and len(slx_) < self.getnumvar():
      raise ValueError("Array argument slx is not long enough")
    if slx_ is None:
      raise ValueError("Argument slx cannot be None")
    if slx_ is None:
      raise ValueError("Argument slx may not be None")
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    res = __library__.MSK_XX_putslx(self.__nativep,whichsol_,_slx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putsux(self,whichsol_,sux_):
    """
    Sets the sux vector for a solution.
  
    putsux(self,whichsol_,sux_)
      whichsol: mosek.soltype. Selects a solution.
      sux: array of double. The sux vector.
    """
    if sux_ is not None and len(sux_) < self.getnumvar():
      raise ValueError("Array argument sux is not long enough")
    if sux_ is None:
      raise ValueError("Argument sux cannot be None")
    if sux_ is None:
      raise ValueError("Argument sux may not be None")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    res = __library__.MSK_XX_putsux(self.__nativep,whichsol_,_sux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_doublevector)
  def putsnx(self,whichsol_,sux_):
    """
    Sets the snx vector for a solution.
  
    putsnx(self,whichsol_,sux_)
      whichsol: mosek.soltype. Selects a solution.
      sux: array of double. The snx vector.
    """
    if sux_ is not None and len(sux_) < self.getnumvar():
      raise ValueError("Array argument sux is not long enough")
    if sux_ is None:
      raise ValueError("Argument sux cannot be None")
    if sux_ is None:
      raise ValueError("Argument sux may not be None")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    res = __library__.MSK_XX_putsnx(self.__nativep,whichsol_,_sux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_anyenumvector(stakey))
  def putskcslice(self,whichsol_,first_,last_,skc_):
    """
    Sets the status keys for the constraints.
  
    putskcslice(self,whichsol_,first_,last_,skc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      skc: array of mosek.stakey. Status keys for the constraints.
    """
    if skc_ is not None and len(skc_) < ((last_) - (first_)):
      raise ValueError("Array argument skc is not long enough")
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))(*skc_)
    else:
        _skc_tmp = None
    res = __library__.MSK_XX_putskcslice(self.__nativep,whichsol_,first_,last_,_skc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_anyenumvector(stakey))
  def putskxslice(self,whichsol_,first_,last_,skx_):
    """
    Sets the status keys for the variables.
  
    putskxslice(self,whichsol_,first_,last_,skx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      skx: array of mosek.stakey. Status keys for the variables.
    """
    if skx_ is not None and len(skx_) < ((last_) - (first_)):
      raise ValueError("Array argument skx is not long enough")
    if skx_ is None:
      raise ValueError("Argument skx cannot be None")
    if skx_ is None:
      raise ValueError("Argument skx may not be None")
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))(*skx_)
    else:
        _skx_tmp = None
    res = __library__.MSK_XX_putskxslice(self.__nativep,whichsol_,first_,last_,_skx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putxcslice(self,whichsol_,first_,last_,xc_):
    """
    Sets a slice of the xc vector for a solution.
  
    putxcslice(self,whichsol_,first_,last_,xc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      xc: array of double. Primal constraint solution.
    """
    if xc_ is not None and len(xc_) < ((last_) - (first_)):
      raise ValueError("Array argument xc is not long enough")
    if xc_ is None:
      raise ValueError("Argument xc cannot be None")
    if xc_ is None:
      raise ValueError("Argument xc may not be None")
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    res = __library__.MSK_XX_putxcslice(self.__nativep,whichsol_,first_,last_,_xc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putxxslice(self,whichsol_,first_,last_,xx_):
    """
    Obtains a slice of the xx vector for a solution.
  
    putxxslice(self,whichsol_,first_,last_,xx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      xx: array of double. Primal variable solution.
    """
    if xx_ is not None and len(xx_) < ((last_) - (first_)):
      raise ValueError("Array argument xx is not long enough")
    if xx_ is None:
      raise ValueError("Argument xx cannot be None")
    if xx_ is None:
      raise ValueError("Argument xx may not be None")
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    res = __library__.MSK_XX_putxxslice(self.__nativep,whichsol_,first_,last_,_xx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putyslice(self,whichsol_,first_,last_,y_):
    """
    Sets a slice of the y vector for a solution.
  
    putyslice(self,whichsol_,first_,last_,y_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      y: array of double. Vector of dual variables corresponding to the constraints.
    """
    if y_ is not None and len(y_) < ((last_) - (first_)):
      raise ValueError("Array argument y is not long enough")
    if y_ is None:
      raise ValueError("Argument y cannot be None")
    if y_ is None:
      raise ValueError("Argument y may not be None")
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    res = __library__.MSK_XX_putyslice(self.__nativep,whichsol_,first_,last_,_y_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putslcslice(self,whichsol_,first_,last_,slc_):
    """
    Sets a slice of the slc vector for a solution.
  
    putslcslice(self,whichsol_,first_,last_,slc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      slc: array of double. Dual variables corresponding to the lower bounds on the constraints.
    """
    if slc_ is not None and len(slc_) < ((last_) - (first_)):
      raise ValueError("Array argument slc is not long enough")
    if slc_ is None:
      raise ValueError("Argument slc cannot be None")
    if slc_ is None:
      raise ValueError("Argument slc may not be None")
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    res = __library__.MSK_XX_putslcslice(self.__nativep,whichsol_,first_,last_,_slc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putsucslice(self,whichsol_,first_,last_,suc_):
    """
    Sets a slice of the suc vector for a solution.
  
    putsucslice(self,whichsol_,first_,last_,suc_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      suc: array of double. Dual variables corresponding to the upper bounds on the constraints.
    """
    if suc_ is not None and len(suc_) < ((last_) - (first_)):
      raise ValueError("Array argument suc is not long enough")
    if suc_ is None:
      raise ValueError("Argument suc cannot be None")
    if suc_ is None:
      raise ValueError("Argument suc may not be None")
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    res = __library__.MSK_XX_putsucslice(self.__nativep,whichsol_,first_,last_,_suc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putslxslice(self,whichsol_,first_,last_,slx_):
    """
    Sets a slice of the slx vector for a solution.
  
    putslxslice(self,whichsol_,first_,last_,slx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      slx: array of double. Dual variables corresponding to the lower bounds on the variables.
    """
    if slx_ is not None and len(slx_) < ((last_) - (first_)):
      raise ValueError("Array argument slx is not long enough")
    if slx_ is None:
      raise ValueError("Argument slx cannot be None")
    if slx_ is None:
      raise ValueError("Argument slx may not be None")
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    res = __library__.MSK_XX_putslxslice(self.__nativep,whichsol_,first_,last_,_slx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putsuxslice(self,whichsol_,first_,last_,sux_):
    """
    Sets a slice of the sux vector for a solution.
  
    putsuxslice(self,whichsol_,first_,last_,sux_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      sux: array of double. Dual variables corresponding to the upper bounds on the variables.
    """
    if sux_ is not None and len(sux_) < ((last_) - (first_)):
      raise ValueError("Array argument sux is not long enough")
    if sux_ is None:
      raise ValueError("Argument sux cannot be None")
    if sux_ is None:
      raise ValueError("Argument sux may not be None")
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    res = __library__.MSK_XX_putsuxslice(self.__nativep,whichsol_,first_,last_,_sux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_make_doublevector)
  def putsnxslice(self,whichsol_,first_,last_,snx_):
    """
    Sets a slice of the snx vector for a solution.
  
    putsnxslice(self,whichsol_,first_,last_,snx_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. First index in the sequence.
      last: int. Last index plus 1 in the sequence.
      snx: array of double. Dual variables corresponding to the conic constraints on the variables.
    """
    if snx_ is not None and len(snx_) < ((last_) - (first_)):
      raise ValueError("Array argument snx is not long enough")
    if snx_ is None:
      raise ValueError("Argument snx cannot be None")
    if snx_ is None:
      raise ValueError("Argument snx may not be None")
    if isinstance(snx_, numpy.ndarray) and snx_.dtype is numpy.dtype(numpy.float64) and snx_.flags.contiguous:
      _snx_copyarray = False
      _snx_tmp = ctypes.cast(snx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(snx_,array.ndarray) and snx_.dtype is numpy.dtype(array.float64) and  snx_.getsteplength() == 1:
      _snx_copyarray = False
      _snx_tmp = snx_.getdatacptr()
    elif snx_ is None:
      _snx_copyarray = False
      _snx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _snx_copyarray = True
      _snx_tmp = (ctypes.c_double * len(snx_))(*snx_)
    res = __library__.MSK_XX_putsnxslice(self.__nativep,whichsol_,first_,last_,_snx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_doublevector)
  def putbarxj(self,whichsol_,j_,barxj_):
    """
    Sets the primal solution for a semidefinite variable.
  
    putbarxj(self,whichsol_,j_,barxj_)
      whichsol: mosek.soltype. Selects a solution.
      j: int. Index of the semidefinite variable.
      barxj: array of double. Value of the j'th variable of barx.
    """
    if barxj_ is not None and len(barxj_) < self.getlenbarvarj((j_)):
      raise ValueError("Array argument barxj is not long enough")
    if barxj_ is None:
      raise ValueError("Argument barxj cannot be None")
    if barxj_ is None:
      raise ValueError("Argument barxj may not be None")
    if isinstance(barxj_, numpy.ndarray) and barxj_.dtype is numpy.dtype(numpy.float64) and barxj_.flags.contiguous:
      _barxj_copyarray = False
      _barxj_tmp = ctypes.cast(barxj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(barxj_,array.ndarray) and barxj_.dtype is numpy.dtype(array.float64) and  barxj_.getsteplength() == 1:
      _barxj_copyarray = False
      _barxj_tmp = barxj_.getdatacptr()
    elif barxj_ is None:
      _barxj_copyarray = False
      _barxj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _barxj_copyarray = True
      _barxj_tmp = (ctypes.c_double * len(barxj_))(*barxj_)
    res = __library__.MSK_XX_putbarxj(self.__nativep,whichsol_,j_,_barxj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_doublevector)
  def putbarsj(self,whichsol_,j_,barsj_):
    """
    Sets the dual solution for a semidefinite variable.
  
    putbarsj(self,whichsol_,j_,barsj_)
      whichsol: mosek.soltype. Selects a solution.
      j: int. Index of the semidefinite variable.
      barsj: array of double. Value of the j'th variable of barx.
    """
    if barsj_ is not None and len(barsj_) < self.getlenbarvarj((j_)):
      raise ValueError("Array argument barsj is not long enough")
    if barsj_ is None:
      raise ValueError("Argument barsj cannot be None")
    if barsj_ is None:
      raise ValueError("Argument barsj may not be None")
    if isinstance(barsj_, numpy.ndarray) and barsj_.dtype is numpy.dtype(numpy.float64) and barsj_.flags.contiguous:
      _barsj_copyarray = False
      _barsj_tmp = ctypes.cast(barsj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(barsj_,array.ndarray) and barsj_.dtype is numpy.dtype(array.float64) and  barsj_.getsteplength() == 1:
      _barsj_copyarray = False
      _barsj_tmp = barsj_.getdatacptr()
    elif barsj_ is None:
      _barsj_copyarray = False
      _barsj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _barsj_copyarray = True
      _barsj_tmp = (ctypes.c_double * len(barsj_))(*barsj_)
    res = __library__.MSK_XX_putbarsj(self.__nativep,whichsol_,j_,_barsj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getsolutioninf(self,whichsol_):
    """
    Deprecated
  
    getsolutioninf(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: prosta,solsta,primalobj,maxpbi,maxpcni,maxpeqi,maxinti,dualobj,maxdbi,maxdcni,maxdeqi
      prosta: mosek.prosta. Problem status.
      solsta: mosek.solsta. Solution status.
      primalobj: double. Value of the primal objective.
      maxpbi: double. Maximum infeasibility in primal bounds on variables.
      maxpcni: double. Maximum infeasibility in the primal conic constraints.
      maxpeqi: double. Maximum infeasibility in primal equality constraints.
      maxinti: double. Maximum infeasibility in primal equality constraints.
      dualobj: double. Value of the dual objective.
      maxdbi: double. Maximum infeasibility in bounds on dual variables.
      maxdcni: double. Maximum infeasibility in the dual conic constraints.
      maxdeqi: double. Maximum infeasibility in the dual equality constraints.
    """
    prosta_ = ctypes.c_int32()
    solsta_ = ctypes.c_int32()
    primalobj_ = ctypes.c_double()
    primalobj_ = ctypes.c_double()
    maxpbi_ = ctypes.c_double()
    maxpbi_ = ctypes.c_double()
    maxpcni_ = ctypes.c_double()
    maxpcni_ = ctypes.c_double()
    maxpeqi_ = ctypes.c_double()
    maxpeqi_ = ctypes.c_double()
    maxinti_ = ctypes.c_double()
    maxinti_ = ctypes.c_double()
    dualobj_ = ctypes.c_double()
    dualobj_ = ctypes.c_double()
    maxdbi_ = ctypes.c_double()
    maxdbi_ = ctypes.c_double()
    maxdcni_ = ctypes.c_double()
    maxdcni_ = ctypes.c_double()
    maxdeqi_ = ctypes.c_double()
    maxdeqi_ = ctypes.c_double()
    res = __library__.MSK_XX_getsolutioninf(self.__nativep,whichsol_,ctypes.byref(prosta_),ctypes.byref(solsta_),ctypes.byref(primalobj_),ctypes.byref(maxpbi_),ctypes.byref(maxpcni_),ctypes.byref(maxpeqi_),ctypes.byref(maxinti_),ctypes.byref(dualobj_),ctypes.byref(maxdbi_),ctypes.byref(maxdcni_),ctypes.byref(maxdeqi_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _prosta_return_value = prosta(prosta_.value)
    _solsta_return_value = solsta(solsta_.value)
    _primalobj_return_value = primalobj_.value
    _maxpbi_return_value = maxpbi_.value
    _maxpcni_return_value = maxpcni_.value
    _maxpeqi_return_value = maxpeqi_.value
    _maxinti_return_value = maxinti_.value
    _dualobj_return_value = dualobj_.value
    _maxdbi_return_value = maxdbi_.value
    _maxdcni_return_value = maxdcni_.value
    _maxdeqi_return_value = maxdeqi_.value
    return (_prosta_return_value,_solsta_return_value,_primalobj_return_value,_maxpbi_return_value,_maxpcni_return_value,_maxpeqi_return_value,_maxinti_return_value,_dualobj_return_value,_maxdbi_return_value,_maxdcni_return_value,_maxdeqi_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getpviolcon(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a primal solution for a list of xc variables.
  
    getpviolcon(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of constraints.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getpviolcon(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getpviolvar(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a primal solution for a list of x variables.
  
    getpviolvar(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of x variables.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getpviolvar(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getpviolbarvar(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a primal solution for a list of barx variables.
  
    getpviolbarvar(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of barx variables.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getpviolbarvar(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getpviolcones(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a solution for set of conic quadratic constraints.
  
    getpviolcones(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of quadratic cones.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getpviolcones(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getdviolcon(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a dual solution associated with a set of constraints.
  
    getdviolcon(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of constraints.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getdviolcon(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getdviolvar(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a dual solution associated with a set of x variables.
  
    getdviolvar(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of x variables.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getdviolvar(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getdviolbarvar(self,whichsol_,sub_,viol_):
    """
    Computes the violation of dual solution for a set of barx variables.
  
    getdviolbarvar(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of barx variables.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getdviolbarvar(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_intvector,_accept_doublevector)
  def getdviolcones(self,whichsol_,sub_,viol_):
    """
    Computes the violation of a solution for set of dual conic quadratic constraints.
  
    getdviolcones(self,whichsol_,sub_,viol_)
      whichsol: mosek.soltype. Selects a solution.
      sub: array of int. An array of indexes of dual quadratic cones.
      viol: array of double. List of violations corresponding to sub.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if viol_ is not None and len(viol_) < (num_):
      raise ValueError("Array argument viol is not long enough")
    if isinstance(viol_,numpy.ndarray) and not viol_.flags.writeable:
      raise ValueError("Argument viol must be writable")
    if viol_ is None:
      raise ValueError("Argument viol may not be None")
    if isinstance(viol_, numpy.ndarray) and viol_.dtype is numpy.dtype(numpy.float64) and viol_.flags.contiguous:
      _viol_copyarray = False
      _viol_tmp = ctypes.cast(viol_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(viol_,array.ndarray) and viol_.dtype is numpy.dtype(array.float64) and  viol_.getsteplength() == 1:
      _viol_copyarray = False
      _viol_tmp = viol_.getdatacptr()
    elif viol_ is None:
      _viol_copyarray = False
      _viol_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _viol_copyarray = True
      _viol_tmp = (ctypes.c_double * len(viol_))(*viol_)
    res = __library__.MSK_XX_getdviolcones(self.__nativep,whichsol_,num_,_sub_tmp,_viol_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _viol_copyarray:
      if isinstance(viol_, numpy.ndarray) and False:
        viol_[:] = list(_viol_tmp)
      else:
        viol_[:] = _viol_tmp
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getsolutioninfo(self,whichsol_):
    """
    Obtains information about of a solution.
  
    getsolutioninfo(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: pobj,pviolcon,pviolvar,pviolbarvar,pviolcone,pviolitg,dobj,dviolcon,dviolvar,dviolbarvar,dviolcone
      pobj: double. The primal objective value.
      pviolcon: double. Maximal primal bound violation for a xc variable.
      pviolvar: double. Maximal primal bound violation for a xx variable.
      pviolbarvar: double. Maximal primal bound violation for a barx variable.
      pviolcone: double. Maximal primal violation of the solution with respect to the conic constraints.
      pviolitg: double. Maximal violation in the integer constraints.
      dobj: double. Dual objective value.
      dviolcon: double. Maximal dual bound violation a xc variable.
      dviolvar: double. Maximal dual bound violation xx variable.
      dviolbarvar: double. Maximal dual bound violation for a bars variable.
      dviolcone: double. Maximum violation of the dual solution in the dual conic constraints .
    """
    pobj_ = ctypes.c_double()
    pobj_ = ctypes.c_double()
    pviolcon_ = ctypes.c_double()
    pviolcon_ = ctypes.c_double()
    pviolvar_ = ctypes.c_double()
    pviolvar_ = ctypes.c_double()
    pviolbarvar_ = ctypes.c_double()
    pviolbarvar_ = ctypes.c_double()
    pviolcone_ = ctypes.c_double()
    pviolcone_ = ctypes.c_double()
    pviolitg_ = ctypes.c_double()
    pviolitg_ = ctypes.c_double()
    dobj_ = ctypes.c_double()
    dobj_ = ctypes.c_double()
    dviolcon_ = ctypes.c_double()
    dviolcon_ = ctypes.c_double()
    dviolvar_ = ctypes.c_double()
    dviolvar_ = ctypes.c_double()
    dviolbarvar_ = ctypes.c_double()
    dviolbarvar_ = ctypes.c_double()
    dviolcone_ = ctypes.c_double()
    dviolcone_ = ctypes.c_double()
    res = __library__.MSK_XX_getsolutioninfo(self.__nativep,whichsol_,ctypes.byref(pobj_),ctypes.byref(pviolcon_),ctypes.byref(pviolvar_),ctypes.byref(pviolbarvar_),ctypes.byref(pviolcone_),ctypes.byref(pviolitg_),ctypes.byref(dobj_),ctypes.byref(dviolcon_),ctypes.byref(dviolvar_),ctypes.byref(dviolbarvar_),ctypes.byref(dviolcone_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _pobj_return_value = pobj_.value
    _pviolcon_return_value = pviolcon_.value
    _pviolvar_return_value = pviolvar_.value
    _pviolbarvar_return_value = pviolbarvar_.value
    _pviolcone_return_value = pviolcone_.value
    _pviolitg_return_value = pviolitg_.value
    _dobj_return_value = dobj_.value
    _dviolcon_return_value = dviolcon_.value
    _dviolvar_return_value = dviolvar_.value
    _dviolbarvar_return_value = dviolbarvar_.value
    _dviolcone_return_value = dviolcone_.value
    return (_pobj_return_value,_pviolcon_return_value,_pviolvar_return_value,_pviolbarvar_return_value,_pviolcone_return_value,_pviolitg_return_value,_dobj_return_value,_dviolcon_return_value,_dviolvar_return_value,_dviolbarvar_return_value,_dviolcone_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_anyenum(solitem),_make_int,_make_int,_accept_doublevector)
  def getsolutionslice(self,whichsol_,solitem_,first_,last_,values_):
    """
    Obtains a slice of the solution.
  
    getsolutionslice(self,whichsol_,solitem_,first_,last_,values_)
      whichsol: mosek.soltype. Selects a solution.
      solitem: mosek.solitem. Which part of the solution is required.
      first: int. Index of the first value in the slice.
      last: int. Value of the last index+1 in the slice.
      values: array of double. The values of the requested solution elements.
    """
    if values_ is not None and len(values_) < ((last_) - (first_)):
      raise ValueError("Array argument values is not long enough")
    if isinstance(values_,numpy.ndarray) and not values_.flags.writeable:
      raise ValueError("Argument values must be writable")
    if isinstance(values_, numpy.ndarray) and values_.dtype is numpy.dtype(numpy.float64) and values_.flags.contiguous:
      _values_copyarray = False
      _values_tmp = ctypes.cast(values_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(values_,array.ndarray) and values_.dtype is numpy.dtype(array.float64) and  values_.getsteplength() == 1:
      _values_copyarray = False
      _values_tmp = values_.getdatacptr()
    elif values_ is None:
      _values_copyarray = False
      _values_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _values_copyarray = True
      _values_tmp = (ctypes.c_double * len(values_))(*values_)
    res = __library__.MSK_XX_getsolutionslice(self.__nativep,whichsol_,solitem_,first_,last_,_values_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _values_copyarray:
      if isinstance(values_, numpy.ndarray) and False:
        values_[:] = list(_values_tmp)
      else:
        values_[:] = _values_tmp
  @accepts(_accept_any,_accept_anyenum(soltype),_make_int,_make_int,_accept_doublevector)
  def getreducedcosts(self,whichsol_,first_,last_,redcosts_):
    """
    Obtains the difference of (slx-sux) for a sequence of variables.
  
    getreducedcosts(self,whichsol_,first_,last_,redcosts_)
      whichsol: mosek.soltype. Selects a solution.
      first: int. See the documentation for a full description.
      last: int. See the documentation for a full description.
      redcosts: array of double. Returns the requested reduced costs. See documentation for a full description.
    """
    if redcosts_ is not None and len(redcosts_) < ((last_) - (first_)):
      raise ValueError("Array argument redcosts is not long enough")
    if isinstance(redcosts_,numpy.ndarray) and not redcosts_.flags.writeable:
      raise ValueError("Argument redcosts must be writable")
    if isinstance(redcosts_, numpy.ndarray) and redcosts_.dtype is numpy.dtype(numpy.float64) and redcosts_.flags.contiguous:
      _redcosts_copyarray = False
      _redcosts_tmp = ctypes.cast(redcosts_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(redcosts_,array.ndarray) and redcosts_.dtype is numpy.dtype(array.float64) and  redcosts_.getsteplength() == 1:
      _redcosts_copyarray = False
      _redcosts_tmp = redcosts_.getdatacptr()
    elif redcosts_ is None:
      _redcosts_copyarray = False
      _redcosts_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _redcosts_copyarray = True
      _redcosts_tmp = (ctypes.c_double * len(redcosts_))(*redcosts_)
    res = __library__.MSK_XX_getreducedcosts(self.__nativep,whichsol_,first_,last_,_redcosts_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _redcosts_copyarray:
      if isinstance(redcosts_, numpy.ndarray) and False:
        redcosts_[:] = list(_redcosts_tmp)
      else:
        redcosts_[:] = _redcosts_tmp
  @accepts(_accept_any,_accept_anyenum(sparam))
  def getstrparam(self,param_):
    """
    Obtains the value of a string parameter.
  
    getstrparam(self,param_)
      param: mosek.sparam. Which parameter.
    returns: len,parvalue
      len: int. The length of the parameter value.
      parvalue: unicode. If this is not NULL, the parameter value is stored here.
    """
    maxlen_ = (1 + self.getstrparamlen((param_)))
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    parvalue_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_getstrparam(self.__nativep,param_,maxlen_,ctypes.byref(len_),parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    _parvalue_retval = parvalue_.value.decode("utf-8")
    return (_len_return_value,_parvalue_retval)
  @accepts(_accept_any,_accept_anyenum(sparam))
  def getstrparamlen(self,param_):
    """
    Obtains the length of a string parameter.
  
    getstrparamlen(self,param_)
      param: mosek.sparam. Which parameter.
    returns: len
      len: int. The length of the parameter value.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_getstrparamlen(self.__nativep,param_,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any)
  def gettasknamelen(self):
    """
    Obtains the length the task name.
  
    gettasknamelen(self)
    returns: len
      len: int. Returns the length of the task name.
    """
    len_ = ctypes.c_int32()
    len_ = ctypes.c_int32()
    res = __library__.MSK_XX_gettasknamelen(self.__nativep,ctypes.byref(len_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _len_return_value = len_.value
    return (_len_return_value)
  @accepts(_accept_any)
  def gettaskname(self):
    """
    Obtains the task name.
  
    gettaskname(self)
    returns: taskname
      taskname: unicode. Is assigned the task name.
    """
    __tmp_var_0 = ctypes.c_int32()
    res = __library__.MSK_XX_gettasknamelen(self.__nativep,ctypes.byref(__tmp_var_0))
    if res != 0:
      result,msg = self.__getlasterror(res)
      raise Error(rescode(result),msg)
    maxlen_ = (1 + __tmp_var_0.value)
    taskname_ = (ctypes.c_char * (maxlen_))()
    res = __library__.MSK_XX_gettaskname(self.__nativep,maxlen_,taskname_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _taskname_retval = taskname_.value.decode("utf-8")
    return (_taskname_retval)
  @accepts(_accept_any,_make_int)
  def getvartype(self,j_):
    """
    Gets the variable type of one variable.
  
    getvartype(self,j_)
      j: int. Index of the variable.
    returns: vartype
      vartype: mosek.variabletype. Variable type of variable index j.
    """
    vartype_ = ctypes.c_int32()
    res = __library__.MSK_XX_getvartype(self.__nativep,j_,ctypes.byref(vartype_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _vartype_return_value = variabletype(vartype_.value)
    return (_vartype_return_value)
  @accepts(_accept_any,_make_intvector,_accept_any)
  def getvartypelist(self,subj_,vartype_):
    """
    Obtains the variable type for one or more variables.
  
    getvartypelist(self,subj_,vartype_)
      subj: array of int. A list of variable indexes.
      vartype: array of mosek.variabletype. Returns the variables types corresponding the variable indexes requested.
    """
    num_ = None
    if num_ is None:
      num_ = len(subj_)
    else:
      num_ = min(num_,len(subj_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if vartype_ is not None and len(vartype_) < (num_):
      raise ValueError("Array argument vartype is not long enough")
    if isinstance(vartype_,numpy.ndarray) and not vartype_.flags.writeable:
      raise ValueError("Argument vartype must be writable")
    if vartype_ is not None:
        _vartype_tmp = (ctypes.c_int32 * len(vartype_))()
    else:
        _vartype_tmp = None
    res = __library__.MSK_XX_getvartypelist(self.__nativep,num_,_subj_tmp,_vartype_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if vartype_ is not None: vartype_[:] = [ variabletype(v) for v in _vartype_tmp[0:len(vartype_)] ]
  @accepts(_accept_any,_make_int,_make_int,_make_doublevector,_make_double,_make_longvector,_make_longvector,_make_intvector,_make_doublevector,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def inputdata(self,maxnumcon_,maxnumvar_,c_,cfix_,aptrb_,aptre_,asub_,aval_,bkc_,blc_,buc_,bkx_,blx_,bux_):
    """
    Input the linear part of an optimization task in one function call.
  
    inputdata(self,maxnumcon_,maxnumvar_,c_,cfix_,aptrb_,aptre_,asub_,aval_,bkc_,blc_,buc_,bkx_,blx_,bux_)
      maxnumcon: int. Number of preallocated constraints in the optimization task.
      maxnumvar: int. Number of preallocated variables in the optimization task.
      c: array of double. Linear terms of the objective as a dense vector. The lengths is the number of variables.
      cfix: double. Fixed term in the objective.
      aptrb: array of long. Row or column end pointers.
      aptre: array of long. Row or column start pointers.
      asub: array of int. Coefficient subscripts.
      aval: array of double. Coefficient coefficient values.
      bkc: array of mosek.boundkey. Bound keys for the constraints.
      blc: array of double. Lower bounds for the constraints.
      buc: array of double. Upper bounds for the constraints.
      bkx: array of mosek.boundkey. Bound keys for the variables.
      blx: array of double. Lower bounds for the variables.
      bux: array of double. Upper bounds for the variables.
    """
    numcon_ = None
    if numcon_ is None:
      numcon_ = len(buc_)
    else:
      numcon_ = min(numcon_,len(buc_))
    if numcon_ is None:
      numcon_ = len(blc_)
    else:
      numcon_ = min(numcon_,len(blc_))
    if numcon_ is None:
      numcon_ = len(bkc_)
    else:
      numcon_ = min(numcon_,len(bkc_))
    numvar_ = None
    if numvar_ is None:
      numvar_ = len(c_)
    else:
      numvar_ = min(numvar_,len(c_))
    if numvar_ is None:
      numvar_ = len(bux_)
    else:
      numvar_ = min(numvar_,len(bux_))
    if numvar_ is None:
      numvar_ = len(blx_)
    else:
      numvar_ = min(numvar_,len(blx_))
    if numvar_ is None:
      numvar_ = len(bkx_)
    else:
      numvar_ = min(numvar_,len(bkx_))
    if numvar_ is None:
      numvar_ = len(aptrb_)
    else:
      numvar_ = min(numvar_,len(aptrb_))
    if numvar_ is None:
      numvar_ = len(aptre_)
    else:
      numvar_ = min(numvar_,len(aptre_))
    if isinstance(c_, numpy.ndarray) and c_.dtype is numpy.dtype(numpy.float64) and c_.flags.contiguous:
      _c_copyarray = False
      _c_tmp = ctypes.cast(c_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(c_,array.ndarray) and c_.dtype is numpy.dtype(array.float64) and  c_.getsteplength() == 1:
      _c_copyarray = False
      _c_tmp = c_.getdatacptr()
    elif c_ is None:
      _c_copyarray = False
      _c_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _c_copyarray = True
      _c_tmp = (ctypes.c_double * len(c_))(*c_)
    if aptrb_ is None:
      raise ValueError("Argument aptrb cannot be None")
    if aptrb_ is None:
      raise ValueError("Argument aptrb may not be None")
    if isinstance(aptrb_, numpy.ndarray) and aptrb_.dtype is numpy.dtype(numpy.int64) and aptrb_.flags.contiguous:
      _aptrb_copyarray = False
      _aptrb_tmp = ctypes.cast(aptrb_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(aptrb_,array.ndarray) and aptrb_.dtype is numpy.dtype(array.int64) and  aptrb_.getsteplength() == 1:
      _aptrb_copyarray = False
      _aptrb_tmp = aptrb_.getdatacptr()
    elif aptrb_ is None:
      _aptrb_copyarray = False
      _aptrb_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _aptrb_copyarray = True
      _aptrb_tmp = (ctypes.c_int64 * len(aptrb_))(*aptrb_)
    if aptre_ is None:
      raise ValueError("Argument aptre cannot be None")
    if aptre_ is None:
      raise ValueError("Argument aptre may not be None")
    if isinstance(aptre_, numpy.ndarray) and aptre_.dtype is numpy.dtype(numpy.int64) and aptre_.flags.contiguous:
      _aptre_copyarray = False
      _aptre_tmp = ctypes.cast(aptre_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(aptre_,array.ndarray) and aptre_.dtype is numpy.dtype(array.int64) and  aptre_.getsteplength() == 1:
      _aptre_copyarray = False
      _aptre_tmp = aptre_.getdatacptr()
    elif aptre_ is None:
      _aptre_copyarray = False
      _aptre_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _aptre_copyarray = True
      _aptre_tmp = (ctypes.c_int64 * len(aptre_))(*aptre_)
    if asub_ is None:
      raise ValueError("Argument asub cannot be None")
    if asub_ is None:
      raise ValueError("Argument asub may not be None")
    if isinstance(asub_, numpy.ndarray) and asub_.dtype is numpy.dtype(numpy.int32) and asub_.flags.contiguous:
      _asub_copyarray = False
      _asub_tmp = ctypes.cast(asub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(asub_,array.ndarray) and asub_.dtype is numpy.dtype(array.int32) and  asub_.getsteplength() == 1:
      _asub_copyarray = False
      _asub_tmp = asub_.getdatacptr()
    elif asub_ is None:
      _asub_copyarray = False
      _asub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _asub_copyarray = True
      _asub_tmp = (ctypes.c_int32 * len(asub_))(*asub_)
    if aval_ is None:
      raise ValueError("Argument aval cannot be None")
    if aval_ is None:
      raise ValueError("Argument aval may not be None")
    if isinstance(aval_, numpy.ndarray) and aval_.dtype is numpy.dtype(numpy.float64) and aval_.flags.contiguous:
      _aval_copyarray = False
      _aval_tmp = ctypes.cast(aval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(aval_,array.ndarray) and aval_.dtype is numpy.dtype(array.float64) and  aval_.getsteplength() == 1:
      _aval_copyarray = False
      _aval_tmp = aval_.getdatacptr()
    elif aval_ is None:
      _aval_copyarray = False
      _aval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _aval_copyarray = True
      _aval_tmp = (ctypes.c_double * len(aval_))(*aval_)
    if bkc_ is None:
      raise ValueError("Argument bkc cannot be None")
    if bkc_ is None:
      raise ValueError("Argument bkc may not be None")
    if bkc_ is not None:
        _bkc_tmp = (ctypes.c_int32 * len(bkc_))(*bkc_)
    else:
        _bkc_tmp = None
    if blc_ is None:
      raise ValueError("Argument blc cannot be None")
    if blc_ is None:
      raise ValueError("Argument blc may not be None")
    if isinstance(blc_, numpy.ndarray) and blc_.dtype is numpy.dtype(numpy.float64) and blc_.flags.contiguous:
      _blc_copyarray = False
      _blc_tmp = ctypes.cast(blc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(blc_,array.ndarray) and blc_.dtype is numpy.dtype(array.float64) and  blc_.getsteplength() == 1:
      _blc_copyarray = False
      _blc_tmp = blc_.getdatacptr()
    elif blc_ is None:
      _blc_copyarray = False
      _blc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _blc_copyarray = True
      _blc_tmp = (ctypes.c_double * len(blc_))(*blc_)
    if buc_ is None:
      raise ValueError("Argument buc cannot be None")
    if buc_ is None:
      raise ValueError("Argument buc may not be None")
    if isinstance(buc_, numpy.ndarray) and buc_.dtype is numpy.dtype(numpy.float64) and buc_.flags.contiguous:
      _buc_copyarray = False
      _buc_tmp = ctypes.cast(buc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(buc_,array.ndarray) and buc_.dtype is numpy.dtype(array.float64) and  buc_.getsteplength() == 1:
      _buc_copyarray = False
      _buc_tmp = buc_.getdatacptr()
    elif buc_ is None:
      _buc_copyarray = False
      _buc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _buc_copyarray = True
      _buc_tmp = (ctypes.c_double * len(buc_))(*buc_)
    if bkx_ is None:
      raise ValueError("Argument bkx cannot be None")
    if bkx_ is None:
      raise ValueError("Argument bkx may not be None")
    if bkx_ is not None:
        _bkx_tmp = (ctypes.c_int32 * len(bkx_))(*bkx_)
    else:
        _bkx_tmp = None
    if blx_ is None:
      raise ValueError("Argument blx cannot be None")
    if blx_ is None:
      raise ValueError("Argument blx may not be None")
    if isinstance(blx_, numpy.ndarray) and blx_.dtype is numpy.dtype(numpy.float64) and blx_.flags.contiguous:
      _blx_copyarray = False
      _blx_tmp = ctypes.cast(blx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(blx_,array.ndarray) and blx_.dtype is numpy.dtype(array.float64) and  blx_.getsteplength() == 1:
      _blx_copyarray = False
      _blx_tmp = blx_.getdatacptr()
    elif blx_ is None:
      _blx_copyarray = False
      _blx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _blx_copyarray = True
      _blx_tmp = (ctypes.c_double * len(blx_))(*blx_)
    if bux_ is None:
      raise ValueError("Argument bux cannot be None")
    if bux_ is None:
      raise ValueError("Argument bux may not be None")
    if isinstance(bux_, numpy.ndarray) and bux_.dtype is numpy.dtype(numpy.float64) and bux_.flags.contiguous:
      _bux_copyarray = False
      _bux_tmp = ctypes.cast(bux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bux_,array.ndarray) and bux_.dtype is numpy.dtype(array.float64) and  bux_.getsteplength() == 1:
      _bux_copyarray = False
      _bux_tmp = bux_.getdatacptr()
    elif bux_ is None:
      _bux_copyarray = False
      _bux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bux_copyarray = True
      _bux_tmp = (ctypes.c_double * len(bux_))(*bux_)
    res = __library__.MSK_XX_inputdata64(self.__nativep,maxnumcon_,maxnumvar_,numcon_,numvar_,_c_tmp,cfix_,_aptrb_tmp,_aptre_tmp,_asub_tmp,_aval_tmp,_bkc_tmp,_blc_tmp,_buc_tmp,_bkx_tmp,_blx_tmp,_bux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def isdouparname(self,parname_):
    """
    Checks a double parameter name.
  
    isdouparname(self,parname_)
      parname: str|unicode. Parameter name.
    returns: param
      param: mosek.dparam. Which parameter.
    """
    if isinstance(parname_,unicode):
      parname_ = parname_.encode("utf-8")
    param_ = ctypes.c_int32()
    res = __library__.MSK_XX_isdouparname(self.__nativep,parname_,ctypes.byref(param_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _param_return_value = dparam(param_.value)
    return (_param_return_value)
  @accepts(_accept_any,_accept_str)
  def isintparname(self,parname_):
    """
    Checks an integer parameter name.
  
    isintparname(self,parname_)
      parname: str|unicode. Parameter name.
    returns: param
      param: mosek.iparam. Which parameter.
    """
    if isinstance(parname_,unicode):
      parname_ = parname_.encode("utf-8")
    param_ = ctypes.c_int32()
    res = __library__.MSK_XX_isintparname(self.__nativep,parname_,ctypes.byref(param_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _param_return_value = iparam(param_.value)
    return (_param_return_value)
  @accepts(_accept_any,_accept_str)
  def isstrparname(self,parname_):
    """
    Checks a string parameter name.
  
    isstrparname(self,parname_)
      parname: str|unicode. Parameter name.
    returns: param
      param: mosek.sparam. Which parameter.
    """
    if isinstance(parname_,unicode):
      parname_ = parname_.encode("utf-8")
    param_ = ctypes.c_int32()
    res = __library__.MSK_XX_isstrparname(self.__nativep,parname_,ctypes.byref(param_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _param_return_value = sparam(param_.value)
    return (_param_return_value)
  @accepts(_accept_any,_accept_anyenum(streamtype),_accept_str,_make_int)
  def linkfiletostream(self,whichstream_,filename_,append_):
    """
    Directs all output from a task stream to a file.
  
    linkfiletostream(self,whichstream_,filename_,append_)
      whichstream: mosek.streamtype. Index of the stream.
      filename: str|unicode. The name of the file where the stream is written.
      append: int. If this argument is 0 the output file will be overwritten, otherwise text is append to the output file.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_linkfiletotaskstream(self.__nativep,whichstream_,filename_,append_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector)
  def relaxprimal(self,wlc_,wuc_,wlx_,wux_):
    """
    Deprecated.
  
    relaxprimal(self,wlc_,wuc_,wlx_,wux_)
      wlc: array of double. Weights associated with lower bounds on the activity of constraints.
      wuc: array of double. Weights associated with upper bounds on                             the activity of constraints.
      wlx: array of double. Weights associated with lower bounds on                             the activity of variables.
      wux: array of double. Weights associated with upper bounds on                             the activity of variables.
    returns: 
    """
    relaxedtask_ = ctypes.c_void_p()
    relaxedtask_ = ctypes.c_void_p()
    if wlc_ is not None and len(wlc_) < self.getnumcon():
      raise ValueError("Array argument wlc is not long enough")
    if isinstance(wlc_,numpy.ndarray) and not wlc_.flags.writeable:
      raise ValueError("Argument wlc must be writable")
    if isinstance(wlc_, numpy.ndarray) and wlc_.dtype is numpy.dtype(numpy.float64) and wlc_.flags.contiguous:
      _wlc_copyarray = False
      _wlc_tmp = ctypes.cast(wlc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wlc_,array.ndarray) and wlc_.dtype is numpy.dtype(array.float64) and  wlc_.getsteplength() == 1:
      _wlc_copyarray = False
      _wlc_tmp = wlc_.getdatacptr()
    elif wlc_ is None:
      _wlc_copyarray = False
      _wlc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wlc_copyarray = True
      _wlc_tmp = (ctypes.c_double * len(wlc_))(*wlc_)
    if wuc_ is not None and len(wuc_) < self.getnumcon():
      raise ValueError("Array argument wuc is not long enough")
    if isinstance(wuc_,numpy.ndarray) and not wuc_.flags.writeable:
      raise ValueError("Argument wuc must be writable")
    if isinstance(wuc_, numpy.ndarray) and wuc_.dtype is numpy.dtype(numpy.float64) and wuc_.flags.contiguous:
      _wuc_copyarray = False
      _wuc_tmp = ctypes.cast(wuc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wuc_,array.ndarray) and wuc_.dtype is numpy.dtype(array.float64) and  wuc_.getsteplength() == 1:
      _wuc_copyarray = False
      _wuc_tmp = wuc_.getdatacptr()
    elif wuc_ is None:
      _wuc_copyarray = False
      _wuc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wuc_copyarray = True
      _wuc_tmp = (ctypes.c_double * len(wuc_))(*wuc_)
    if wlx_ is not None and len(wlx_) < self.getnumvar():
      raise ValueError("Array argument wlx is not long enough")
    if isinstance(wlx_,numpy.ndarray) and not wlx_.flags.writeable:
      raise ValueError("Argument wlx must be writable")
    if isinstance(wlx_, numpy.ndarray) and wlx_.dtype is numpy.dtype(numpy.float64) and wlx_.flags.contiguous:
      _wlx_copyarray = False
      _wlx_tmp = ctypes.cast(wlx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wlx_,array.ndarray) and wlx_.dtype is numpy.dtype(array.float64) and  wlx_.getsteplength() == 1:
      _wlx_copyarray = False
      _wlx_tmp = wlx_.getdatacptr()
    elif wlx_ is None:
      _wlx_copyarray = False
      _wlx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wlx_copyarray = True
      _wlx_tmp = (ctypes.c_double * len(wlx_))(*wlx_)
    if wux_ is not None and len(wux_) < self.getnumvar():
      raise ValueError("Array argument wux is not long enough")
    if isinstance(wux_,numpy.ndarray) and not wux_.flags.writeable:
      raise ValueError("Argument wux must be writable")
    if isinstance(wux_, numpy.ndarray) and wux_.dtype is numpy.dtype(numpy.float64) and wux_.flags.contiguous:
      _wux_copyarray = False
      _wux_tmp = ctypes.cast(wux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wux_,array.ndarray) and wux_.dtype is numpy.dtype(array.float64) and  wux_.getsteplength() == 1:
      _wux_copyarray = False
      _wux_tmp = wux_.getdatacptr()
    elif wux_ is None:
      _wux_copyarray = False
      _wux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wux_copyarray = True
      _wux_tmp = (ctypes.c_double * len(wux_))(*wux_)
    res = __library__.MSK_XX_relaxprimal(self.__nativep,ctypes.byref(relaxedtask_),_wlc_tmp,_wuc_tmp,_wlx_tmp,_wux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _relaxedtask_return_value = Task(nativep = relaxedtask_.value)
    if _wlc_copyarray:
      if isinstance(wlc_, numpy.ndarray) and False:
        wlc_[:] = list(_wlc_tmp)
      else:
        wlc_[:] = _wlc_tmp
    if _wuc_copyarray:
      if isinstance(wuc_, numpy.ndarray) and False:
        wuc_[:] = list(_wuc_tmp)
      else:
        wuc_[:] = _wuc_tmp
    if _wlx_copyarray:
      if isinstance(wlx_, numpy.ndarray) and False:
        wlx_[:] = list(_wlx_tmp)
      else:
        wlx_[:] = _wlx_tmp
    if _wux_copyarray:
      if isinstance(wux_, numpy.ndarray) and False:
        wux_[:] = list(_wux_tmp)
      else:
        wux_[:] = _wux_tmp
    return (_relaxedtask_return_value)
  @accepts(_accept_any,_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector)
  def primalrepair(self,wlc_,wuc_,wlx_,wux_):
    """
    The function repairs a primal infeasible optimization problem by adjusting the bounds on the constraints and variables.
  
    primalrepair(self,wlc_,wuc_,wlx_,wux_)
      wlc: array of double. Weights associated with relaxing lower bounds on the constraints.
      wuc: array of double. Weights associated with relaxing the upper bound on the constraints.
      wlx: array of double. Weights associated with relaxing the lower bounds of the variables.
      wux: array of double. Weights associated with relaxing the upper bounds of variables.
    """
    if wlc_ is not None and len(wlc_) < self.getnumcon():
      raise ValueError("Array argument wlc is not long enough")
    if isinstance(wlc_, numpy.ndarray) and wlc_.dtype is numpy.dtype(numpy.float64) and wlc_.flags.contiguous:
      _wlc_copyarray = False
      _wlc_tmp = ctypes.cast(wlc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wlc_,array.ndarray) and wlc_.dtype is numpy.dtype(array.float64) and  wlc_.getsteplength() == 1:
      _wlc_copyarray = False
      _wlc_tmp = wlc_.getdatacptr()
    elif wlc_ is None:
      _wlc_copyarray = False
      _wlc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wlc_copyarray = True
      _wlc_tmp = (ctypes.c_double * len(wlc_))(*wlc_)
    if wuc_ is not None and len(wuc_) < self.getnumcon():
      raise ValueError("Array argument wuc is not long enough")
    if isinstance(wuc_, numpy.ndarray) and wuc_.dtype is numpy.dtype(numpy.float64) and wuc_.flags.contiguous:
      _wuc_copyarray = False
      _wuc_tmp = ctypes.cast(wuc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wuc_,array.ndarray) and wuc_.dtype is numpy.dtype(array.float64) and  wuc_.getsteplength() == 1:
      _wuc_copyarray = False
      _wuc_tmp = wuc_.getdatacptr()
    elif wuc_ is None:
      _wuc_copyarray = False
      _wuc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wuc_copyarray = True
      _wuc_tmp = (ctypes.c_double * len(wuc_))(*wuc_)
    if wlx_ is not None and len(wlx_) < self.getnumvar():
      raise ValueError("Array argument wlx is not long enough")
    if isinstance(wlx_, numpy.ndarray) and wlx_.dtype is numpy.dtype(numpy.float64) and wlx_.flags.contiguous:
      _wlx_copyarray = False
      _wlx_tmp = ctypes.cast(wlx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wlx_,array.ndarray) and wlx_.dtype is numpy.dtype(array.float64) and  wlx_.getsteplength() == 1:
      _wlx_copyarray = False
      _wlx_tmp = wlx_.getdatacptr()
    elif wlx_ is None:
      _wlx_copyarray = False
      _wlx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wlx_copyarray = True
      _wlx_tmp = (ctypes.c_double * len(wlx_))(*wlx_)
    if wux_ is not None and len(wux_) < self.getnumvar():
      raise ValueError("Array argument wux is not long enough")
    if isinstance(wux_, numpy.ndarray) and wux_.dtype is numpy.dtype(numpy.float64) and wux_.flags.contiguous:
      _wux_copyarray = False
      _wux_tmp = ctypes.cast(wux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(wux_,array.ndarray) and wux_.dtype is numpy.dtype(array.float64) and  wux_.getsteplength() == 1:
      _wux_copyarray = False
      _wux_tmp = wux_.getdatacptr()
    elif wux_ is None:
      _wux_copyarray = False
      _wux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _wux_copyarray = True
      _wux_tmp = (ctypes.c_double * len(wux_))(*wux_)
    res = __library__.MSK_XX_primalrepair(self.__nativep,_wlc_tmp,_wuc_tmp,_wlx_tmp,_wux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def toconic(self):
    """
    Inplace reformulation of a QCQP to a COP
  
    toconic(self)
    """
    res = __library__.MSK_XX_toconic(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_check_taskvector)
  def optimizeconcurrent(self,taskarray_):
    """
    Optimize a given task with several optimizers concurrently.
  
    optimizeconcurrent(self,taskarray_)
      taskarray: array of mosek.Task. An array of tasks.
    """
    if taskarray_ is None:
      raise ValueError("Argument taskarray cannot be None")
    if taskarray_ is None:
      raise ValueError("Argument taskarray may not be None")
    if taskarray_ is None:
      _taskarray_tmp = None
    else:
      _taskarray_tmp = (ctypes.c_void_p * len(taskarray_))(*[ ((i is not None and i.__nativep) or None) for i in taskarray_ ])
    num_ = None
    if num_ is None:
      num_ = len(taskarray_)
    else:
      num_ = min(num_,len(taskarray_))
    res = __library__.MSK_XX_optimizeconcurrent(self.__nativep,_taskarray_tmp,num_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def optimize(self):
    """
    Optimizes the problem.
  
    optimize(self)
    returns: trmcode
      trmcode: mosek.rescode. Is either OK or a termination response code.
    """
    trmcode_ = ctypes.c_int32()
    res = __library__.MSK_XX_optimizetrm(self.__nativep,ctypes.byref(trmcode_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _trmcode_return_value = rescode(trmcode_.value)
    return (_trmcode_return_value)
  @accepts(_accept_any,_accept_anyenum(streamtype),_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int,_make_int)
  def printdata(self,whichstream_,firsti_,lasti_,firstj_,lastj_,firstk_,lastk_,c_,qo_,a_,qc_,bc_,bx_,vartype_,cones_):
    """
    Prints a part of the problem data to a stream.
  
    printdata(self,whichstream_,firsti_,lasti_,firstj_,lastj_,firstk_,lastk_,c_,qo_,a_,qc_,bc_,bx_,vartype_,cones_)
      whichstream: mosek.streamtype. Index of the stream.
      firsti: int. Index of first constraint for which data should be printed.
      lasti: int. Index of last constraint plus 1 for which data should be printed.
      firstj: int. Index of first variable for which data should be printed.
      lastj: int. Index of last variable plus 1 for which data should be printed.
      firstk: int. Index of first cone for which data should be printed.
      lastk: int. Index of last cone plus 1 for which data should be printed.
      c: int. If non-zero the linear objective terms are printed.
      qo: int. If non-zero the quadratic objective terms are printed.
      a: int. If non-zero the linear constraint matrix is printed.
      qc: int. If non-zero q'th quadratic constraint terms are printed for the relevant constraints.
      bc: int. If non-zero the constraints bounds are printed.
      bx: int. If non-zero the variable bounds are printed.
      vartype: int. If non-zero the variable types are printed.
      cones: int. If non-zero the  conic data is printed.
    """
    res = __library__.MSK_XX_printdata(self.__nativep,whichstream_,firsti_,lasti_,firstj_,lastj_,firstk_,lastk_,c_,qo_,a_,qc_,bc_,bx_,vartype_,cones_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def printparam(self):
    """
    Prints the current parameter settings.
  
    printparam(self)
    """
    res = __library__.MSK_XX_printparam(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def commitchanges(self):
    """
    Commits all cached problem changes.
  
    commitchanges(self)
    """
    res = __library__.MSK_XX_commitchanges(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_double)
  def putaij(self,i_,j_,aij_):
    """
    Changes a single value in the linear coefficient matrix.
  
    putaij(self,i_,j_,aij_)
      i: int. Index of the constraint in which the change should occur.
      j: int. Index of the variable in which the change should occur.
      aij: double. New coefficient.
    """
    res = __library__.MSK_XX_putaij(self.__nativep,i_,j_,aij_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_intvector,_make_doublevector)
  def putaijlist(self,subi_,subj_,valij_):
    """
    Changes one or more coefficients in the linear constraint matrix.
  
    putaijlist(self,subi_,subj_,valij_)
      subi: array of int. Constraint indexes in which the change should occur.
      subj: array of int. Variable indexes in which the change should occur.
      valij: array of double. New coefficient values.
    """
    num_ = None
    if num_ is None:
      num_ = len(subi_)
    else:
      num_ = min(num_,len(subi_))
    if num_ is None:
      num_ = len(subj_)
    else:
      num_ = min(num_,len(subj_))
    if num_ is None:
      num_ = len(valij_)
    else:
      num_ = min(num_,len(valij_))
    if subi_ is None:
      raise ValueError("Argument subi cannot be None")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if valij_ is None:
      raise ValueError("Argument valij cannot be None")
    if valij_ is None:
      raise ValueError("Argument valij may not be None")
    if isinstance(valij_, numpy.ndarray) and valij_.dtype is numpy.dtype(numpy.float64) and valij_.flags.contiguous:
      _valij_copyarray = False
      _valij_tmp = ctypes.cast(valij_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valij_,array.ndarray) and valij_.dtype is numpy.dtype(array.float64) and  valij_.getsteplength() == 1:
      _valij_copyarray = False
      _valij_tmp = valij_.getdatacptr()
    elif valij_ is None:
      _valij_copyarray = False
      _valij_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valij_copyarray = True
      _valij_tmp = (ctypes.c_double * len(valij_))(*valij_)
    res = __library__.MSK_XX_putaijlist64(self.__nativep,num_,_subi_tmp,_subj_tmp,_valij_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_intvector,_make_doublevector)
  def putacol(self,j_,subj_,valj_):
    """
    Replaces all elements in one column of A.
  
    putacol(self,j_,subj_,valj_)
      j: int. Column index.
      subj: array of int. Row indexes of non-zero values in column.
      valj: array of double. New non-zero values of column.
    """
    nzj_ = None
    if nzj_ is None:
      nzj_ = len(subj_)
    else:
      nzj_ = min(nzj_,len(subj_))
    if nzj_ is None:
      nzj_ = len(valj_)
    else:
      nzj_ = min(nzj_,len(valj_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if valj_ is None:
      raise ValueError("Argument valj cannot be None")
    if valj_ is None:
      raise ValueError("Argument valj may not be None")
    if isinstance(valj_, numpy.ndarray) and valj_.dtype is numpy.dtype(numpy.float64) and valj_.flags.contiguous:
      _valj_copyarray = False
      _valj_tmp = ctypes.cast(valj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valj_,array.ndarray) and valj_.dtype is numpy.dtype(array.float64) and  valj_.getsteplength() == 1:
      _valj_copyarray = False
      _valj_tmp = valj_.getdatacptr()
    elif valj_ is None:
      _valj_copyarray = False
      _valj_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valj_copyarray = True
      _valj_tmp = (ctypes.c_double * len(valj_))(*valj_)
    res = __library__.MSK_XX_putacol(self.__nativep,j_,nzj_,_subj_tmp,_valj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_intvector,_make_doublevector)
  def putarow(self,i_,subi_,vali_):
    """
    Replaces all elements in one row of A.
  
    putarow(self,i_,subi_,vali_)
      i: int. row index.
      subi: array of int. Row indexes of non-zero values in row.
      vali: array of double. New non-zero values of row.
    """
    nzi_ = None
    if nzi_ is None:
      nzi_ = len(subi_)
    else:
      nzi_ = min(nzi_,len(subi_))
    if nzi_ is None:
      nzi_ = len(vali_)
    else:
      nzi_ = min(nzi_,len(vali_))
    if subi_ is None:
      raise ValueError("Argument subi cannot be None")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if vali_ is None:
      raise ValueError("Argument vali cannot be None")
    if vali_ is None:
      raise ValueError("Argument vali may not be None")
    if isinstance(vali_, numpy.ndarray) and vali_.dtype is numpy.dtype(numpy.float64) and vali_.flags.contiguous:
      _vali_copyarray = False
      _vali_tmp = ctypes.cast(vali_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(vali_,array.ndarray) and vali_.dtype is numpy.dtype(array.float64) and  vali_.getsteplength() == 1:
      _vali_copyarray = False
      _vali_tmp = vali_.getdatacptr()
    elif vali_ is None:
      _vali_copyarray = False
      _vali_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _vali_copyarray = True
      _vali_tmp = (ctypes.c_double * len(vali_))(*vali_)
    res = __library__.MSK_XX_putarow(self.__nativep,i_,nzi_,_subi_tmp,_vali_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_longvector,_make_longvector,_make_intvector,_make_doublevector)
  def putarowlist(self,sub_,ptrb_,ptre_,asub_,aval_):
    """
    Replaces all elements in several rows the linear constraint matrix by new values.
  
    putarowlist(self,sub_,ptrb_,ptre_,asub_,aval_)
      sub: array of int. Indexes of rows or columns that should be replaced.
      ptrb: array of long. Array of pointers to the first element in the rows or columns.
      ptre: array of long. Array of pointers to the last element plus one in the rows or columns.
      asub: array of int. Variable indexes.
      aval: array of double. Coefficient coefficient values.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(ptrb_)
    else:
      num_ = min(num_,len(ptrb_))
    if num_ is None:
      num_ = len(ptre_)
    else:
      num_ = min(num_,len(ptre_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if ptrb_ is None:
      raise ValueError("Argument ptrb cannot be None")
    if ptrb_ is None:
      raise ValueError("Argument ptrb may not be None")
    if isinstance(ptrb_, numpy.ndarray) and ptrb_.dtype is numpy.dtype(numpy.int64) and ptrb_.flags.contiguous:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.cast(ptrb_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptrb_,array.ndarray) and ptrb_.dtype is numpy.dtype(array.int64) and  ptrb_.getsteplength() == 1:
      _ptrb_copyarray = False
      _ptrb_tmp = ptrb_.getdatacptr()
    elif ptrb_ is None:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptrb_copyarray = True
      _ptrb_tmp = (ctypes.c_int64 * len(ptrb_))(*ptrb_)
    if ptre_ is None:
      raise ValueError("Argument ptre cannot be None")
    if ptre_ is None:
      raise ValueError("Argument ptre may not be None")
    if isinstance(ptre_, numpy.ndarray) and ptre_.dtype is numpy.dtype(numpy.int64) and ptre_.flags.contiguous:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.cast(ptre_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptre_,array.ndarray) and ptre_.dtype is numpy.dtype(array.int64) and  ptre_.getsteplength() == 1:
      _ptre_copyarray = False
      _ptre_tmp = ptre_.getdatacptr()
    elif ptre_ is None:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptre_copyarray = True
      _ptre_tmp = (ctypes.c_int64 * len(ptre_))(*ptre_)
    if asub_ is None:
      raise ValueError("Argument asub cannot be None")
    if asub_ is None:
      raise ValueError("Argument asub may not be None")
    if isinstance(asub_, numpy.ndarray) and asub_.dtype is numpy.dtype(numpy.int32) and asub_.flags.contiguous:
      _asub_copyarray = False
      _asub_tmp = ctypes.cast(asub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(asub_,array.ndarray) and asub_.dtype is numpy.dtype(array.int32) and  asub_.getsteplength() == 1:
      _asub_copyarray = False
      _asub_tmp = asub_.getdatacptr()
    elif asub_ is None:
      _asub_copyarray = False
      _asub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _asub_copyarray = True
      _asub_tmp = (ctypes.c_int32 * len(asub_))(*asub_)
    if aval_ is None:
      raise ValueError("Argument aval cannot be None")
    if aval_ is None:
      raise ValueError("Argument aval may not be None")
    if isinstance(aval_, numpy.ndarray) and aval_.dtype is numpy.dtype(numpy.float64) and aval_.flags.contiguous:
      _aval_copyarray = False
      _aval_tmp = ctypes.cast(aval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(aval_,array.ndarray) and aval_.dtype is numpy.dtype(array.float64) and  aval_.getsteplength() == 1:
      _aval_copyarray = False
      _aval_tmp = aval_.getdatacptr()
    elif aval_ is None:
      _aval_copyarray = False
      _aval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _aval_copyarray = True
      _aval_tmp = (ctypes.c_double * len(aval_))(*aval_)
    res = __library__.MSK_XX_putarowlist64(self.__nativep,num_,_sub_tmp,_ptrb_tmp,_ptre_tmp,_asub_tmp,_aval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_longvector,_make_longvector,_make_intvector,_make_doublevector)
  def putacolslice(self,first_,last_,ptrb_,ptre_,asub_,aval_):
    """
    Replaces all elements in several columns the linear constraint matrix by new values.
  
    putacolslice(self,first_,last_,ptrb_,ptre_,asub_,aval_)
      first: int. First column in the slice.
      last: int. Last column plus one in the slice.
      ptrb: array of long. Array of pointers to the first element in the columns.
      ptre: array of long. Array of pointers to the last element plus one in the columns.
      asub: array of int. Variable indexes.
      aval: array of double. Coefficient coefficient values.
    """
    if ptrb_ is not None and len(ptrb_) < ((last_) - (first_)):
      raise ValueError("Array argument ptrb is not long enough")
    if ptrb_ is None:
      raise ValueError("Argument ptrb cannot be None")
    if ptrb_ is None:
      raise ValueError("Argument ptrb may not be None")
    if isinstance(ptrb_, numpy.ndarray) and ptrb_.dtype is numpy.dtype(numpy.int64) and ptrb_.flags.contiguous:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.cast(ptrb_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptrb_,array.ndarray) and ptrb_.dtype is numpy.dtype(array.int64) and  ptrb_.getsteplength() == 1:
      _ptrb_copyarray = False
      _ptrb_tmp = ptrb_.getdatacptr()
    elif ptrb_ is None:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptrb_copyarray = True
      _ptrb_tmp = (ctypes.c_int64 * len(ptrb_))(*ptrb_)
    if ptre_ is not None and len(ptre_) < ((last_) - (first_)):
      raise ValueError("Array argument ptre is not long enough")
    if ptre_ is None:
      raise ValueError("Argument ptre cannot be None")
    if ptre_ is None:
      raise ValueError("Argument ptre may not be None")
    if isinstance(ptre_, numpy.ndarray) and ptre_.dtype is numpy.dtype(numpy.int64) and ptre_.flags.contiguous:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.cast(ptre_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptre_,array.ndarray) and ptre_.dtype is numpy.dtype(array.int64) and  ptre_.getsteplength() == 1:
      _ptre_copyarray = False
      _ptre_tmp = ptre_.getdatacptr()
    elif ptre_ is None:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptre_copyarray = True
      _ptre_tmp = (ctypes.c_int64 * len(ptre_))(*ptre_)
    if asub_ is None:
      raise ValueError("Argument asub cannot be None")
    if asub_ is None:
      raise ValueError("Argument asub may not be None")
    if isinstance(asub_, numpy.ndarray) and asub_.dtype is numpy.dtype(numpy.int32) and asub_.flags.contiguous:
      _asub_copyarray = False
      _asub_tmp = ctypes.cast(asub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(asub_,array.ndarray) and asub_.dtype is numpy.dtype(array.int32) and  asub_.getsteplength() == 1:
      _asub_copyarray = False
      _asub_tmp = asub_.getdatacptr()
    elif asub_ is None:
      _asub_copyarray = False
      _asub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _asub_copyarray = True
      _asub_tmp = (ctypes.c_int32 * len(asub_))(*asub_)
    if aval_ is None:
      raise ValueError("Argument aval cannot be None")
    if aval_ is None:
      raise ValueError("Argument aval may not be None")
    if isinstance(aval_, numpy.ndarray) and aval_.dtype is numpy.dtype(numpy.float64) and aval_.flags.contiguous:
      _aval_copyarray = False
      _aval_tmp = ctypes.cast(aval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(aval_,array.ndarray) and aval_.dtype is numpy.dtype(array.float64) and  aval_.getsteplength() == 1:
      _aval_copyarray = False
      _aval_tmp = aval_.getdatacptr()
    elif aval_ is None:
      _aval_copyarray = False
      _aval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _aval_copyarray = True
      _aval_tmp = (ctypes.c_double * len(aval_))(*aval_)
    res = __library__.MSK_XX_putacolslice64(self.__nativep,first_,last_,_ptrb_tmp,_ptre_tmp,_asub_tmp,_aval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_longvector,_make_longvector,_make_intvector,_make_doublevector)
  def putacollist(self,sub_,ptrb_,ptre_,asub_,aval_):
    """
    Replaces all elements in several columns the linear constraint matrix by new values.
  
    putacollist(self,sub_,ptrb_,ptre_,asub_,aval_)
      sub: array of int. Indexes of columns that should be replaced.
      ptrb: array of long. Array of pointers to the first element in the columns.
      ptre: array of long. Array of pointers to the last element plus one in the columns.
      asub: array of int. Variable indexes.
      aval: array of double. Coefficient coefficient values.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(ptrb_)
    else:
      num_ = min(num_,len(ptrb_))
    if num_ is None:
      num_ = len(ptre_)
    else:
      num_ = min(num_,len(ptre_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if ptrb_ is None:
      raise ValueError("Argument ptrb cannot be None")
    if ptrb_ is None:
      raise ValueError("Argument ptrb may not be None")
    if isinstance(ptrb_, numpy.ndarray) and ptrb_.dtype is numpy.dtype(numpy.int64) and ptrb_.flags.contiguous:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.cast(ptrb_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptrb_,array.ndarray) and ptrb_.dtype is numpy.dtype(array.int64) and  ptrb_.getsteplength() == 1:
      _ptrb_copyarray = False
      _ptrb_tmp = ptrb_.getdatacptr()
    elif ptrb_ is None:
      _ptrb_copyarray = False
      _ptrb_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptrb_copyarray = True
      _ptrb_tmp = (ctypes.c_int64 * len(ptrb_))(*ptrb_)
    if ptre_ is None:
      raise ValueError("Argument ptre cannot be None")
    if ptre_ is None:
      raise ValueError("Argument ptre may not be None")
    if isinstance(ptre_, numpy.ndarray) and ptre_.dtype is numpy.dtype(numpy.int64) and ptre_.flags.contiguous:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.cast(ptre_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(ptre_,array.ndarray) and ptre_.dtype is numpy.dtype(array.int64) and  ptre_.getsteplength() == 1:
      _ptre_copyarray = False
      _ptre_tmp = ptre_.getdatacptr()
    elif ptre_ is None:
      _ptre_copyarray = False
      _ptre_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _ptre_copyarray = True
      _ptre_tmp = (ctypes.c_int64 * len(ptre_))(*ptre_)
    if asub_ is None:
      raise ValueError("Argument asub cannot be None")
    if asub_ is None:
      raise ValueError("Argument asub may not be None")
    if isinstance(asub_, numpy.ndarray) and asub_.dtype is numpy.dtype(numpy.int32) and asub_.flags.contiguous:
      _asub_copyarray = False
      _asub_tmp = ctypes.cast(asub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(asub_,array.ndarray) and asub_.dtype is numpy.dtype(array.int32) and  asub_.getsteplength() == 1:
      _asub_copyarray = False
      _asub_tmp = asub_.getdatacptr()
    elif asub_ is None:
      _asub_copyarray = False
      _asub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _asub_copyarray = True
      _asub_tmp = (ctypes.c_int32 * len(asub_))(*asub_)
    if aval_ is None:
      raise ValueError("Argument aval cannot be None")
    if aval_ is None:
      raise ValueError("Argument aval may not be None")
    if isinstance(aval_, numpy.ndarray) and aval_.dtype is numpy.dtype(numpy.float64) and aval_.flags.contiguous:
      _aval_copyarray = False
      _aval_tmp = ctypes.cast(aval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(aval_,array.ndarray) and aval_.dtype is numpy.dtype(array.float64) and  aval_.getsteplength() == 1:
      _aval_copyarray = False
      _aval_tmp = aval_.getdatacptr()
    elif aval_ is None:
      _aval_copyarray = False
      _aval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _aval_copyarray = True
      _aval_tmp = (ctypes.c_double * len(aval_))(*aval_)
    res = __library__.MSK_XX_putacollist64(self.__nativep,num_,_sub_tmp,_ptrb_tmp,_ptre_tmp,_asub_tmp,_aval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_longvector,_make_doublevector)
  def putbaraij(self,i_,j_,sub_,weights_):
    """
    Inputs an element of barA.
  
    putbaraij(self,i_,j_,sub_,weights_)
      i: int. Row index of barA.
      j: int. Column index of barA.
      sub: array of long. See argument weights for an explenation.
      weights: array of double. Weights in the weighted sum.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(weights_)
    else:
      num_ = min(num_,len(weights_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int64) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int64) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int64 * len(sub_))(*sub_)
    if weights_ is None:
      raise ValueError("Argument weights cannot be None")
    if weights_ is None:
      raise ValueError("Argument weights may not be None")
    if isinstance(weights_, numpy.ndarray) and weights_.dtype is numpy.dtype(numpy.float64) and weights_.flags.contiguous:
      _weights_copyarray = False
      _weights_tmp = ctypes.cast(weights_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(weights_,array.ndarray) and weights_.dtype is numpy.dtype(array.float64) and  weights_.getsteplength() == 1:
      _weights_copyarray = False
      _weights_tmp = weights_.getdatacptr()
    elif weights_ is None:
      _weights_copyarray = False
      _weights_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _weights_copyarray = True
      _weights_tmp = (ctypes.c_double * len(weights_))(*weights_)
    res = __library__.MSK_XX_putbaraij(self.__nativep,i_,j_,num_,_sub_tmp,_weights_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getnumbarcnz(self):
    """
    Obtains the number of nonzero elements in barc.
  
    getnumbarcnz(self)
    returns: nz
      nz: long. The number of nonzero elements in barc.
    """
    nz_ = ctypes.c_int64()
    nz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumbarcnz(self.__nativep,ctypes.byref(nz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nz_return_value = nz_.value
    return (_nz_return_value)
  @accepts(_accept_any)
  def getnumbaranz(self):
    """
    Get the number of nonzero elements in barA.
  
    getnumbaranz(self)
    returns: nz
      nz: long. The number of nonzero block elements in barA.
    """
    nz_ = ctypes.c_int64()
    nz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumbaranz(self.__nativep,ctypes.byref(nz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _nz_return_value = nz_.value
    return (_nz_return_value)
  @accepts(_accept_any,_accept_longvector)
  def getbarcsparsity(self,idxj_):
    """
    Get the positions of the nonzero elements in barc.
  
    getbarcsparsity(self,idxj_)
      idxj: array of long. Internal positions of the nonzeros elements in barc.
    returns: numnz
      numnz: long. Number of nonzero elements in barc.
    """
    maxnumnz_ = self.getnumbarcnz()
    numnz_ = ctypes.c_int64()
    numnz_ = ctypes.c_int64()
    if idxj_ is not None and len(idxj_) < (maxnumnz_):
      raise ValueError("Array argument idxj is not long enough")
    if isinstance(idxj_,numpy.ndarray) and not idxj_.flags.writeable:
      raise ValueError("Argument idxj must be writable")
    if idxj_ is None:
      raise ValueError("Argument idxj may not be None")
    if isinstance(idxj_, numpy.ndarray) and idxj_.dtype is numpy.dtype(numpy.int64) and idxj_.flags.contiguous:
      _idxj_copyarray = False
      _idxj_tmp = ctypes.cast(idxj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(idxj_,array.ndarray) and idxj_.dtype is numpy.dtype(array.int64) and  idxj_.getsteplength() == 1:
      _idxj_copyarray = False
      _idxj_tmp = idxj_.getdatacptr()
    elif idxj_ is None:
      _idxj_copyarray = False
      _idxj_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _idxj_copyarray = True
      _idxj_tmp = (ctypes.c_int64 * len(idxj_))(*idxj_)
    res = __library__.MSK_XX_getbarcsparsity(self.__nativep,maxnumnz_,ctypes.byref(numnz_),_idxj_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numnz_return_value = numnz_.value
    if _idxj_copyarray:
      if isinstance(idxj_, numpy.ndarray) and False:
        idxj_[:] = list(_idxj_tmp)
      else:
        idxj_[:] = _idxj_tmp
    return (_numnz_return_value)
  @accepts(_accept_any,_accept_longvector)
  def getbarasparsity(self,idxij_):
    """
    Obtains the sparsity pattern of the barA matrix.
  
    getbarasparsity(self,idxij_)
      idxij: array of long. Position of each nonzero element in the vector representation of barA.
    returns: numnz
      numnz: long. Number of nonzero elements in barA.
    """
    maxnumnz_ = self.getnumbaranz()
    numnz_ = ctypes.c_int64()
    numnz_ = ctypes.c_int64()
    if idxij_ is not None and len(idxij_) < (maxnumnz_):
      raise ValueError("Array argument idxij is not long enough")
    if isinstance(idxij_,numpy.ndarray) and not idxij_.flags.writeable:
      raise ValueError("Argument idxij must be writable")
    if idxij_ is None:
      raise ValueError("Argument idxij may not be None")
    if isinstance(idxij_, numpy.ndarray) and idxij_.dtype is numpy.dtype(numpy.int64) and idxij_.flags.contiguous:
      _idxij_copyarray = False
      _idxij_tmp = ctypes.cast(idxij_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(idxij_,array.ndarray) and idxij_.dtype is numpy.dtype(array.int64) and  idxij_.getsteplength() == 1:
      _idxij_copyarray = False
      _idxij_tmp = idxij_.getdatacptr()
    elif idxij_ is None:
      _idxij_copyarray = False
      _idxij_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _idxij_copyarray = True
      _idxij_tmp = (ctypes.c_int64 * len(idxij_))(*idxij_)
    res = __library__.MSK_XX_getbarasparsity(self.__nativep,maxnumnz_,ctypes.byref(numnz_),_idxij_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _numnz_return_value = numnz_.value
    if _idxij_copyarray:
      if isinstance(idxij_, numpy.ndarray) and False:
        idxij_[:] = list(_idxij_tmp)
      else:
        idxij_[:] = _idxij_tmp
    return (_numnz_return_value)
  @accepts(_accept_any,_make_long)
  def getbarcidxinfo(self,idx_):
    """
    Obtains information about an element in barc.
  
    getbarcidxinfo(self,idx_)
      idx: long. Index of element that should be obtained information about. The value is an index of a symmetric sparse variable.
    returns: num
      num: long. Number of terms that appears in weighted that forms the requested element.
    """
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    res = __library__.MSK_XX_getbarcidxinfo(self.__nativep,idx_,ctypes.byref(num_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    return (_num_return_value)
  @accepts(_accept_any,_make_long)
  def getbarcidxj(self,idx_):
    """
    Obtains the row index of an element in barc.
  
    getbarcidxj(self,idx_)
      idx: long. Index of the element that should be obtained information about.
    returns: j
      j: int. Row index in barc.
    """
    j_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    res = __library__.MSK_XX_getbarcidxj(self.__nativep,idx_,ctypes.byref(j_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _j_return_value = j_.value
    return (_j_return_value)
  @accepts(_accept_any,_make_long,_accept_longvector,_accept_doublevector)
  def getbarcidx(self,idx_,sub_,weights_):
    """
    Obtains information about an element in barc.
  
    getbarcidx(self,idx_,sub_,weights_)
      idx: long. Index of the element that should be obtained information about.
      sub: array of long. Elements appearing the weighted sum.
      weights: array of double. Weights of terms in the weighted sum.
    returns: j,num
      j: int. Row index in barc.
      num: long. Number of terms in the weighted sum.
    """
    maxnum_ = self.getbarcidxinfo((idx_))
    j_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    if sub_ is not None and len(sub_) < (maxnum_):
      raise ValueError("Array argument sub is not long enough")
    if isinstance(sub_,numpy.ndarray) and not sub_.flags.writeable:
      raise ValueError("Argument sub must be writable")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int64) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int64) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int64 * len(sub_))(*sub_)
    if weights_ is not None and len(weights_) < (maxnum_):
      raise ValueError("Array argument weights is not long enough")
    if isinstance(weights_,numpy.ndarray) and not weights_.flags.writeable:
      raise ValueError("Argument weights must be writable")
    if weights_ is None:
      raise ValueError("Argument weights may not be None")
    if isinstance(weights_, numpy.ndarray) and weights_.dtype is numpy.dtype(numpy.float64) and weights_.flags.contiguous:
      _weights_copyarray = False
      _weights_tmp = ctypes.cast(weights_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(weights_,array.ndarray) and weights_.dtype is numpy.dtype(array.float64) and  weights_.getsteplength() == 1:
      _weights_copyarray = False
      _weights_tmp = weights_.getdatacptr()
    elif weights_ is None:
      _weights_copyarray = False
      _weights_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _weights_copyarray = True
      _weights_tmp = (ctypes.c_double * len(weights_))(*weights_)
    res = __library__.MSK_XX_getbarcidx(self.__nativep,idx_,maxnum_,ctypes.byref(j_),ctypes.byref(num_),_sub_tmp,_weights_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _j_return_value = j_.value
    _num_return_value = num_.value
    if _sub_copyarray:
      if isinstance(sub_, numpy.ndarray) and False:
        sub_[:] = list(_sub_tmp)
      else:
        sub_[:] = _sub_tmp
    if _weights_copyarray:
      if isinstance(weights_, numpy.ndarray) and False:
        weights_[:] = list(_weights_tmp)
      else:
        weights_[:] = _weights_tmp
    return (_j_return_value,_num_return_value)
  @accepts(_accept_any,_make_long)
  def getbaraidxinfo(self,idx_):
    """
    Obtains the number terms in the weighted sum that forms a particular element in barA.
  
    getbaraidxinfo(self,idx_)
      idx: long. The internal position of the element that should be obtained information for.
    returns: num
      num: long. Number of terms in the weighted sum that forms the specified element in barA.
    """
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    res = __library__.MSK_XX_getbaraidxinfo(self.__nativep,idx_,ctypes.byref(num_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    return (_num_return_value)
  @accepts(_accept_any,_make_long)
  def getbaraidxij(self,idx_):
    """
    Obtains information about an element barA.
  
    getbaraidxij(self,idx_)
      idx: long. Position of the element in the vectorized form.
    returns: i,j
      i: int. Row index of the element at position idx.
      j: int. Column index of the element at position idx.
    """
    i_ = ctypes.c_int32()
    i_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    res = __library__.MSK_XX_getbaraidxij(self.__nativep,idx_,ctypes.byref(i_),ctypes.byref(j_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _i_return_value = i_.value
    _j_return_value = j_.value
    return (_i_return_value,_j_return_value)
  @accepts(_accept_any,_make_long,_accept_longvector,_accept_doublevector)
  def getbaraidx(self,idx_,sub_,weights_):
    """
    Obtains information about an element barA.
  
    getbaraidx(self,idx_,sub_,weights_)
      idx: long. Position of the element in the vectorized form.
      sub: array of long. A list indexes of the elements from symmetric matrix storage that appers in the weighted sum.
      weights: array of double. The weights associated with each term in the weighted sum.
    returns: i,j,num
      i: int. Row index of the element at position idx.
      j: int. Column index of the element at position idx.
      num: long. Number of terms in weighted sum that forms the element.
    """
    maxnum_ = self.getbaraidxinfo((idx_))
    i_ = ctypes.c_int32()
    i_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    j_ = ctypes.c_int32()
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    if sub_ is not None and len(sub_) < (maxnum_):
      raise ValueError("Array argument sub is not long enough")
    if isinstance(sub_,numpy.ndarray) and not sub_.flags.writeable:
      raise ValueError("Argument sub must be writable")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int64) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int64) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int64 * len(sub_))(*sub_)
    if weights_ is not None and len(weights_) < (maxnum_):
      raise ValueError("Array argument weights is not long enough")
    if isinstance(weights_,numpy.ndarray) and not weights_.flags.writeable:
      raise ValueError("Argument weights must be writable")
    if weights_ is None:
      raise ValueError("Argument weights may not be None")
    if isinstance(weights_, numpy.ndarray) and weights_.dtype is numpy.dtype(numpy.float64) and weights_.flags.contiguous:
      _weights_copyarray = False
      _weights_tmp = ctypes.cast(weights_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(weights_,array.ndarray) and weights_.dtype is numpy.dtype(array.float64) and  weights_.getsteplength() == 1:
      _weights_copyarray = False
      _weights_tmp = weights_.getdatacptr()
    elif weights_ is None:
      _weights_copyarray = False
      _weights_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _weights_copyarray = True
      _weights_tmp = (ctypes.c_double * len(weights_))(*weights_)
    res = __library__.MSK_XX_getbaraidx(self.__nativep,idx_,maxnum_,ctypes.byref(i_),ctypes.byref(j_),ctypes.byref(num_),_sub_tmp,_weights_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _i_return_value = i_.value
    _j_return_value = j_.value
    _num_return_value = num_.value
    if _sub_copyarray:
      if isinstance(sub_, numpy.ndarray) and False:
        sub_[:] = list(_sub_tmp)
      else:
        sub_[:] = _sub_tmp
    if _weights_copyarray:
      if isinstance(weights_, numpy.ndarray) and False:
        weights_[:] = list(_weights_tmp)
      else:
        weights_[:] = _weights_tmp
    return (_i_return_value,_j_return_value,_num_return_value)
  @accepts(_accept_any,_make_long,_make_intvector,_make_intvector,_make_intvector,_make_doublevector)
  def putbarcblocktriplet(self,num_,subj_,subk_,subl_,valjkl_):
    """
    Inputs barC in block triplet form.
  
    putbarcblocktriplet(self,num_,subj_,subk_,subl_,valjkl_)
      num: long. Number of elements in the block triplet form.
      subj: array of int. Symmetric matrix variable index.
      subk: array of int. Block row index.
      subl: array of int. Block column index.
      valjkl: array of double. The numerical value associated with the block triplet.
    """
    if subj_ is not None and len(subj_) < (num_):
      raise ValueError("Array argument subj is not long enough")
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if subk_ is not None and len(subk_) < (num_):
      raise ValueError("Array argument subk is not long enough")
    if subk_ is None:
      raise ValueError("Argument subk cannot be None")
    if subk_ is None:
      raise ValueError("Argument subk may not be None")
    if isinstance(subk_, numpy.ndarray) and subk_.dtype is numpy.dtype(numpy.int32) and subk_.flags.contiguous:
      _subk_copyarray = False
      _subk_tmp = ctypes.cast(subk_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subk_,array.ndarray) and subk_.dtype is numpy.dtype(array.int32) and  subk_.getsteplength() == 1:
      _subk_copyarray = False
      _subk_tmp = subk_.getdatacptr()
    elif subk_ is None:
      _subk_copyarray = False
      _subk_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subk_copyarray = True
      _subk_tmp = (ctypes.c_int32 * len(subk_))(*subk_)
    if subl_ is not None and len(subl_) < (num_):
      raise ValueError("Array argument subl is not long enough")
    if subl_ is None:
      raise ValueError("Argument subl cannot be None")
    if subl_ is None:
      raise ValueError("Argument subl may not be None")
    if isinstance(subl_, numpy.ndarray) and subl_.dtype is numpy.dtype(numpy.int32) and subl_.flags.contiguous:
      _subl_copyarray = False
      _subl_tmp = ctypes.cast(subl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subl_,array.ndarray) and subl_.dtype is numpy.dtype(array.int32) and  subl_.getsteplength() == 1:
      _subl_copyarray = False
      _subl_tmp = subl_.getdatacptr()
    elif subl_ is None:
      _subl_copyarray = False
      _subl_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subl_copyarray = True
      _subl_tmp = (ctypes.c_int32 * len(subl_))(*subl_)
    if valjkl_ is not None and len(valjkl_) < (num_):
      raise ValueError("Array argument valjkl is not long enough")
    if valjkl_ is None:
      raise ValueError("Argument valjkl cannot be None")
    if valjkl_ is None:
      raise ValueError("Argument valjkl may not be None")
    if isinstance(valjkl_, numpy.ndarray) and valjkl_.dtype is numpy.dtype(numpy.float64) and valjkl_.flags.contiguous:
      _valjkl_copyarray = False
      _valjkl_tmp = ctypes.cast(valjkl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valjkl_,array.ndarray) and valjkl_.dtype is numpy.dtype(array.float64) and  valjkl_.getsteplength() == 1:
      _valjkl_copyarray = False
      _valjkl_tmp = valjkl_.getdatacptr()
    elif valjkl_ is None:
      _valjkl_copyarray = False
      _valjkl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valjkl_copyarray = True
      _valjkl_tmp = (ctypes.c_double * len(valjkl_))(*valjkl_)
    res = __library__.MSK_XX_putbarcblocktriplet(self.__nativep,num_,_subj_tmp,_subk_tmp,_subl_tmp,_valjkl_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_long,_make_intvector,_make_intvector,_make_intvector,_make_intvector,_make_doublevector)
  def putbarablocktriplet(self,num_,subi_,subj_,subk_,subl_,valijkl_):
    """
    Inputs barA in block triplet form.
  
    putbarablocktriplet(self,num_,subi_,subj_,subk_,subl_,valijkl_)
      num: long. Number of elements in the block triplet form.
      subi: array of int. Constraint index.
      subj: array of int. Symmetric matrix variable index.
      subk: array of int. Block row index.
      subl: array of int. Block column index.
      valijkl: array of double. The numerical value associated with the block triplet.
    """
    if subi_ is not None and len(subi_) < (num_):
      raise ValueError("Array argument subi is not long enough")
    if subi_ is None:
      raise ValueError("Argument subi cannot be None")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is not None and len(subj_) < (num_):
      raise ValueError("Array argument subj is not long enough")
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if subk_ is not None and len(subk_) < (num_):
      raise ValueError("Array argument subk is not long enough")
    if subk_ is None:
      raise ValueError("Argument subk cannot be None")
    if subk_ is None:
      raise ValueError("Argument subk may not be None")
    if isinstance(subk_, numpy.ndarray) and subk_.dtype is numpy.dtype(numpy.int32) and subk_.flags.contiguous:
      _subk_copyarray = False
      _subk_tmp = ctypes.cast(subk_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subk_,array.ndarray) and subk_.dtype is numpy.dtype(array.int32) and  subk_.getsteplength() == 1:
      _subk_copyarray = False
      _subk_tmp = subk_.getdatacptr()
    elif subk_ is None:
      _subk_copyarray = False
      _subk_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subk_copyarray = True
      _subk_tmp = (ctypes.c_int32 * len(subk_))(*subk_)
    if subl_ is not None and len(subl_) < (num_):
      raise ValueError("Array argument subl is not long enough")
    if subl_ is None:
      raise ValueError("Argument subl cannot be None")
    if subl_ is None:
      raise ValueError("Argument subl may not be None")
    if isinstance(subl_, numpy.ndarray) and subl_.dtype is numpy.dtype(numpy.int32) and subl_.flags.contiguous:
      _subl_copyarray = False
      _subl_tmp = ctypes.cast(subl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subl_,array.ndarray) and subl_.dtype is numpy.dtype(array.int32) and  subl_.getsteplength() == 1:
      _subl_copyarray = False
      _subl_tmp = subl_.getdatacptr()
    elif subl_ is None:
      _subl_copyarray = False
      _subl_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subl_copyarray = True
      _subl_tmp = (ctypes.c_int32 * len(subl_))(*subl_)
    if valijkl_ is not None and len(valijkl_) < (num_):
      raise ValueError("Array argument valijkl is not long enough")
    if valijkl_ is None:
      raise ValueError("Argument valijkl cannot be None")
    if valijkl_ is None:
      raise ValueError("Argument valijkl may not be None")
    if isinstance(valijkl_, numpy.ndarray) and valijkl_.dtype is numpy.dtype(numpy.float64) and valijkl_.flags.contiguous:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.cast(valijkl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valijkl_,array.ndarray) and valijkl_.dtype is numpy.dtype(array.float64) and  valijkl_.getsteplength() == 1:
      _valijkl_copyarray = False
      _valijkl_tmp = valijkl_.getdatacptr()
    elif valijkl_ is None:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valijkl_copyarray = True
      _valijkl_tmp = (ctypes.c_double * len(valijkl_))(*valijkl_)
    res = __library__.MSK_XX_putbarablocktriplet(self.__nativep,num_,_subi_tmp,_subj_tmp,_subk_tmp,_subl_tmp,_valijkl_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getnumbarcblocktriplets(self):
    """
    Obtains an upper bound on the number of elements in the block triplet form of barc.
  
    getnumbarcblocktriplets(self)
    returns: num
      num: long. An upper bound on the number elements in the block trip let form of barc.
    """
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumbarcblocktriplets(self.__nativep,ctypes.byref(num_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    return (_num_return_value)
  @accepts(_accept_any,_accept_intvector,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getbarcblocktriplet(self,subj_,subk_,subl_,valijkl_):
    """
    Obtains barc in block triplet form.
  
    getbarcblocktriplet(self,subj_,subk_,subl_,valijkl_)
      subj: array of int. Symmetric matrix variable index.
      subk: array of int. Block row index.
      subl: array of int. Block column index.
      valijkl: array of double. A list indexes of the elements from symmetric matrix storage that appers in the weighted sum.
    returns: num
      num: long. Number of elements in the block triplet form.
    """
    maxnum_ = self.getnumbarcblocktriplets()
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    if subj_ is not None and len(subj_) < (maxnum_):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if subk_ is not None and len(subk_) < (maxnum_):
      raise ValueError("Array argument subk is not long enough")
    if isinstance(subk_,numpy.ndarray) and not subk_.flags.writeable:
      raise ValueError("Argument subk must be writable")
    if subk_ is None:
      raise ValueError("Argument subk may not be None")
    if isinstance(subk_, numpy.ndarray) and subk_.dtype is numpy.dtype(numpy.int32) and subk_.flags.contiguous:
      _subk_copyarray = False
      _subk_tmp = ctypes.cast(subk_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subk_,array.ndarray) and subk_.dtype is numpy.dtype(array.int32) and  subk_.getsteplength() == 1:
      _subk_copyarray = False
      _subk_tmp = subk_.getdatacptr()
    elif subk_ is None:
      _subk_copyarray = False
      _subk_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subk_copyarray = True
      _subk_tmp = (ctypes.c_int32 * len(subk_))(*subk_)
    if subl_ is not None and len(subl_) < (maxnum_):
      raise ValueError("Array argument subl is not long enough")
    if isinstance(subl_,numpy.ndarray) and not subl_.flags.writeable:
      raise ValueError("Argument subl must be writable")
    if subl_ is None:
      raise ValueError("Argument subl may not be None")
    if isinstance(subl_, numpy.ndarray) and subl_.dtype is numpy.dtype(numpy.int32) and subl_.flags.contiguous:
      _subl_copyarray = False
      _subl_tmp = ctypes.cast(subl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subl_,array.ndarray) and subl_.dtype is numpy.dtype(array.int32) and  subl_.getsteplength() == 1:
      _subl_copyarray = False
      _subl_tmp = subl_.getdatacptr()
    elif subl_ is None:
      _subl_copyarray = False
      _subl_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subl_copyarray = True
      _subl_tmp = (ctypes.c_int32 * len(subl_))(*subl_)
    if valijkl_ is not None and len(valijkl_) < (maxnum_):
      raise ValueError("Array argument valijkl is not long enough")
    if isinstance(valijkl_,numpy.ndarray) and not valijkl_.flags.writeable:
      raise ValueError("Argument valijkl must be writable")
    if valijkl_ is None:
      raise ValueError("Argument valijkl may not be None")
    if isinstance(valijkl_, numpy.ndarray) and valijkl_.dtype is numpy.dtype(numpy.float64) and valijkl_.flags.contiguous:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.cast(valijkl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valijkl_,array.ndarray) and valijkl_.dtype is numpy.dtype(array.float64) and  valijkl_.getsteplength() == 1:
      _valijkl_copyarray = False
      _valijkl_tmp = valijkl_.getdatacptr()
    elif valijkl_ is None:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valijkl_copyarray = True
      _valijkl_tmp = (ctypes.c_double * len(valijkl_))(*valijkl_)
    res = __library__.MSK_XX_getbarcblocktriplet(self.__nativep,maxnum_,ctypes.byref(num_),_subj_tmp,_subk_tmp,_subl_tmp,_valijkl_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _subk_copyarray:
      if isinstance(subk_, numpy.ndarray) and False:
        subk_[:] = list(_subk_tmp)
      else:
        subk_[:] = _subk_tmp
    if _subl_copyarray:
      if isinstance(subl_, numpy.ndarray) and False:
        subl_[:] = list(_subl_tmp)
      else:
        subl_[:] = _subl_tmp
    if _valijkl_copyarray:
      if isinstance(valijkl_, numpy.ndarray) and False:
        valijkl_[:] = list(_valijkl_tmp)
      else:
        valijkl_[:] = _valijkl_tmp
    return (_num_return_value)
  @accepts(_accept_any)
  def getnumbarablocktriplets(self):
    """
    Obtains an upper bound on the number of scalar elements in the block triplet form of bara.
  
    getnumbarablocktriplets(self)
    returns: num
      num: long. Number elements in the block triplet form of bara.
    """
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumbarablocktriplets(self.__nativep,ctypes.byref(num_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    return (_num_return_value)
  @accepts(_accept_any,_accept_intvector,_accept_intvector,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getbarablocktriplet(self,subi_,subj_,subk_,subl_,valijkl_):
    """
    Obtains barA in block triplet form.
  
    getbarablocktriplet(self,subi_,subj_,subk_,subl_,valijkl_)
      subi: array of int. Constraint index.
      subj: array of int. Symmetric matrix variable index.
      subk: array of int. Block row index.
      subl: array of int. Block column index.
      valijkl: array of double. A list indexes of the elements from symmetric matrix storage that appers in the weighted sum.
    returns: num
      num: long. Number of elements in the block triplet form.
    """
    maxnum_ = self.getnumbarablocktriplets()
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    if subi_ is not None and len(subi_) < (maxnum_):
      raise ValueError("Array argument subi is not long enough")
    if isinstance(subi_,numpy.ndarray) and not subi_.flags.writeable:
      raise ValueError("Argument subi must be writable")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is not None and len(subj_) < (maxnum_):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if subk_ is not None and len(subk_) < (maxnum_):
      raise ValueError("Array argument subk is not long enough")
    if isinstance(subk_,numpy.ndarray) and not subk_.flags.writeable:
      raise ValueError("Argument subk must be writable")
    if subk_ is None:
      raise ValueError("Argument subk may not be None")
    if isinstance(subk_, numpy.ndarray) and subk_.dtype is numpy.dtype(numpy.int32) and subk_.flags.contiguous:
      _subk_copyarray = False
      _subk_tmp = ctypes.cast(subk_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subk_,array.ndarray) and subk_.dtype is numpy.dtype(array.int32) and  subk_.getsteplength() == 1:
      _subk_copyarray = False
      _subk_tmp = subk_.getdatacptr()
    elif subk_ is None:
      _subk_copyarray = False
      _subk_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subk_copyarray = True
      _subk_tmp = (ctypes.c_int32 * len(subk_))(*subk_)
    if subl_ is not None and len(subl_) < (maxnum_):
      raise ValueError("Array argument subl is not long enough")
    if isinstance(subl_,numpy.ndarray) and not subl_.flags.writeable:
      raise ValueError("Argument subl must be writable")
    if subl_ is None:
      raise ValueError("Argument subl may not be None")
    if isinstance(subl_, numpy.ndarray) and subl_.dtype is numpy.dtype(numpy.int32) and subl_.flags.contiguous:
      _subl_copyarray = False
      _subl_tmp = ctypes.cast(subl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subl_,array.ndarray) and subl_.dtype is numpy.dtype(array.int32) and  subl_.getsteplength() == 1:
      _subl_copyarray = False
      _subl_tmp = subl_.getdatacptr()
    elif subl_ is None:
      _subl_copyarray = False
      _subl_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subl_copyarray = True
      _subl_tmp = (ctypes.c_int32 * len(subl_))(*subl_)
    if valijkl_ is not None and len(valijkl_) < (maxnum_):
      raise ValueError("Array argument valijkl is not long enough")
    if isinstance(valijkl_,numpy.ndarray) and not valijkl_.flags.writeable:
      raise ValueError("Argument valijkl must be writable")
    if valijkl_ is None:
      raise ValueError("Argument valijkl may not be None")
    if isinstance(valijkl_, numpy.ndarray) and valijkl_.dtype is numpy.dtype(numpy.float64) and valijkl_.flags.contiguous:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.cast(valijkl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valijkl_,array.ndarray) and valijkl_.dtype is numpy.dtype(array.float64) and  valijkl_.getsteplength() == 1:
      _valijkl_copyarray = False
      _valijkl_tmp = valijkl_.getdatacptr()
    elif valijkl_ is None:
      _valijkl_copyarray = False
      _valijkl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valijkl_copyarray = True
      _valijkl_tmp = (ctypes.c_double * len(valijkl_))(*valijkl_)
    res = __library__.MSK_XX_getbarablocktriplet(self.__nativep,maxnum_,ctypes.byref(num_),_subi_tmp,_subj_tmp,_subk_tmp,_subl_tmp,_valijkl_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    if _subi_copyarray:
      if isinstance(subi_, numpy.ndarray) and False:
        subi_[:] = list(_subi_tmp)
      else:
        subi_[:] = _subi_tmp
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _subk_copyarray:
      if isinstance(subk_, numpy.ndarray) and False:
        subk_[:] = list(_subk_tmp)
      else:
        subk_[:] = _subk_tmp
    if _subl_copyarray:
      if isinstance(subl_, numpy.ndarray) and False:
        subl_[:] = list(_subl_tmp)
      else:
        subl_[:] = _subl_tmp
    if _valijkl_copyarray:
      if isinstance(valijkl_, numpy.ndarray) and False:
        valijkl_[:] = list(_valijkl_tmp)
      else:
        valijkl_[:] = _valijkl_tmp
    return (_num_return_value)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_accept_anyenum(boundkey),_make_double,_make_double)
  def putbound(self,accmode_,i_,bk_,bl_,bu_):
    """
    Changes the bound for either one constraint or one variable.
  
    putbound(self,accmode_,i_,bk_,bl_,bu_)
      accmode: mosek.accmode. Defines whether the bound for a constraint or a variable is changed.
      i: int. Index of the constraint or variable.
      bk: mosek.boundkey. New bound key.
      bl: double. New lower bound.
      bu: double. New upper bound.
    """
    res = __library__.MSK_XX_putbound(self.__nativep,accmode_,i_,bk_,bl_,bu_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_intvector,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putboundlist(self,accmode_,sub_,bk_,bl_,bu_):
    """
    Changes the bounds of constraints or variables.
  
    putboundlist(self,accmode_,sub_,bk_,bl_,bu_)
      accmode: mosek.accmode. Defines whether to access bounds on variables or constraints.
      sub: array of int. Subscripts of the bounds that should be changed.
      bk: array of mosek.boundkey. Bound keys for variables or constraints.
      bl: array of double. Bound keys for variables or constraints.
      bu: array of double. Constraint or variable upper bounds.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(bk_)
    else:
      num_ = min(num_,len(bk_))
    if num_ is None:
      num_ = len(bl_)
    else:
      num_ = min(num_,len(bl_))
    if num_ is None:
      num_ = len(bu_)
    else:
      num_ = min(num_,len(bu_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if bk_ is None:
      raise ValueError("Argument bk cannot be None")
    if bk_ is None:
      raise ValueError("Argument bk may not be None")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))(*bk_)
    else:
        _bk_tmp = None
    if bl_ is None:
      raise ValueError("Argument bl cannot be None")
    if bl_ is None:
      raise ValueError("Argument bl may not be None")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is None:
      raise ValueError("Argument bu cannot be None")
    if bu_ is None:
      raise ValueError("Argument bu may not be None")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_putboundlist(self.__nativep,accmode_,num_,_sub_tmp,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_anyenum(boundkey),_make_double,_make_double)
  def putconbound(self,i_,bk_,bl_,bu_):
    """
    Changes the bound for one constraint.
  
    putconbound(self,i_,bk_,bl_,bu_)
      i: int. Index of the constraint.
      bk: mosek.boundkey. New bound key.
      bl: double. New lower bound.
      bu: double. New upper bound.
    """
    res = __library__.MSK_XX_putconbound(self.__nativep,i_,bk_,bl_,bu_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putconboundlist(self,sub_,bkc_,blc_,buc_):
    """
    Changes the bounds of a list of constraints.
  
    putconboundlist(self,sub_,bkc_,blc_,buc_)
      sub: array of int. List constraints indexes.
      bkc: array of mosek.boundkey. New bound keys.
      blc: array of double. New lower bound values.
      buc: array of double. New upper bounds values.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(bkc_)
    else:
      num_ = min(num_,len(bkc_))
    if num_ is None:
      num_ = len(blc_)
    else:
      num_ = min(num_,len(blc_))
    if num_ is None:
      num_ = len(buc_)
    else:
      num_ = min(num_,len(buc_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if bkc_ is None:
      raise ValueError("Argument bkc cannot be None")
    if bkc_ is None:
      raise ValueError("Argument bkc may not be None")
    if bkc_ is not None:
        _bkc_tmp = (ctypes.c_int32 * len(bkc_))(*bkc_)
    else:
        _bkc_tmp = None
    if blc_ is None:
      raise ValueError("Argument blc cannot be None")
    if blc_ is None:
      raise ValueError("Argument blc may not be None")
    if isinstance(blc_, numpy.ndarray) and blc_.dtype is numpy.dtype(numpy.float64) and blc_.flags.contiguous:
      _blc_copyarray = False
      _blc_tmp = ctypes.cast(blc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(blc_,array.ndarray) and blc_.dtype is numpy.dtype(array.float64) and  blc_.getsteplength() == 1:
      _blc_copyarray = False
      _blc_tmp = blc_.getdatacptr()
    elif blc_ is None:
      _blc_copyarray = False
      _blc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _blc_copyarray = True
      _blc_tmp = (ctypes.c_double * len(blc_))(*blc_)
    if buc_ is None:
      raise ValueError("Argument buc cannot be None")
    if buc_ is None:
      raise ValueError("Argument buc may not be None")
    if isinstance(buc_, numpy.ndarray) and buc_.dtype is numpy.dtype(numpy.float64) and buc_.flags.contiguous:
      _buc_copyarray = False
      _buc_tmp = ctypes.cast(buc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(buc_,array.ndarray) and buc_.dtype is numpy.dtype(array.float64) and  buc_.getsteplength() == 1:
      _buc_copyarray = False
      _buc_tmp = buc_.getdatacptr()
    elif buc_ is None:
      _buc_copyarray = False
      _buc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _buc_copyarray = True
      _buc_tmp = (ctypes.c_double * len(buc_))(*buc_)
    res = __library__.MSK_XX_putconboundlist(self.__nativep,num_,_sub_tmp,_bkc_tmp,_blc_tmp,_buc_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putconboundslice(self,first_,last_,bk_,bl_,bu_):
    """
    Changes the bounds for a slice of the constraints.
  
    putconboundslice(self,first_,last_,bk_,bl_,bu_)
      first: int. Index of the first constraint in the slice.
      last: int. Index of the last constraint in the slice plus 1.
      bk: array of mosek.boundkey. New bound keys.
      bl: array of double. New lower bounds.
      bu: array of double. New upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if bk_ is None:
      raise ValueError("Argument bk cannot be None")
    if bk_ is None:
      raise ValueError("Argument bk may not be None")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))(*bk_)
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if bl_ is None:
      raise ValueError("Argument bl cannot be None")
    if bl_ is None:
      raise ValueError("Argument bl may not be None")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if bu_ is None:
      raise ValueError("Argument bu cannot be None")
    if bu_ is None:
      raise ValueError("Argument bu may not be None")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_putconboundslice(self.__nativep,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_anyenum(boundkey),_make_double,_make_double)
  def putvarbound(self,j_,bk_,bl_,bu_):
    """
    Changes the bound for one variable.
  
    putvarbound(self,j_,bk_,bl_,bu_)
      j: int. Index of the variable.
      bk: mosek.boundkey. New bound key.
      bl: double. New lower bound.
      bu: double. New upper bound.
    """
    res = __library__.MSK_XX_putvarbound(self.__nativep,j_,bk_,bl_,bu_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putvarboundlist(self,sub_,bkx_,blx_,bux_):
    """
    Changes the bounds of a list of variables.
  
    putvarboundlist(self,sub_,bkx_,blx_,bux_)
      sub: array of int. List of variable indexes.
      bkx: array of mosek.boundkey. New bound keys.
      blx: array of double. New lower bound values.
      bux: array of double. New upper bounds values.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(bkx_)
    else:
      num_ = min(num_,len(bkx_))
    if num_ is None:
      num_ = len(blx_)
    else:
      num_ = min(num_,len(blx_))
    if num_ is None:
      num_ = len(bux_)
    else:
      num_ = min(num_,len(bux_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int32) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int32) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int32 * len(sub_))(*sub_)
    if bkx_ is None:
      raise ValueError("Argument bkx cannot be None")
    if bkx_ is None:
      raise ValueError("Argument bkx may not be None")
    if bkx_ is not None:
        _bkx_tmp = (ctypes.c_int32 * len(bkx_))(*bkx_)
    else:
        _bkx_tmp = None
    if blx_ is None:
      raise ValueError("Argument blx cannot be None")
    if blx_ is None:
      raise ValueError("Argument blx may not be None")
    if isinstance(blx_, numpy.ndarray) and blx_.dtype is numpy.dtype(numpy.float64) and blx_.flags.contiguous:
      _blx_copyarray = False
      _blx_tmp = ctypes.cast(blx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(blx_,array.ndarray) and blx_.dtype is numpy.dtype(array.float64) and  blx_.getsteplength() == 1:
      _blx_copyarray = False
      _blx_tmp = blx_.getdatacptr()
    elif blx_ is None:
      _blx_copyarray = False
      _blx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _blx_copyarray = True
      _blx_tmp = (ctypes.c_double * len(blx_))(*blx_)
    if bux_ is None:
      raise ValueError("Argument bux cannot be None")
    if bux_ is None:
      raise ValueError("Argument bux may not be None")
    if isinstance(bux_, numpy.ndarray) and bux_.dtype is numpy.dtype(numpy.float64) and bux_.flags.contiguous:
      _bux_copyarray = False
      _bux_tmp = ctypes.cast(bux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bux_,array.ndarray) and bux_.dtype is numpy.dtype(array.float64) and  bux_.getsteplength() == 1:
      _bux_copyarray = False
      _bux_tmp = bux_.getdatacptr()
    elif bux_ is None:
      _bux_copyarray = False
      _bux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bux_copyarray = True
      _bux_tmp = (ctypes.c_double * len(bux_))(*bux_)
    res = __library__.MSK_XX_putvarboundlist(self.__nativep,num_,_sub_tmp,_bkx_tmp,_blx_tmp,_bux_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_anyenumvector(boundkey),_make_doublevector,_make_doublevector)
  def putvarboundslice(self,first_,last_,bk_,bl_,bu_):
    """
    Changes the bounds for a slice of the variables.
  
    putvarboundslice(self,first_,last_,bk_,bl_,bu_)
      first: int. Index of the first variable in the slice.
      last: int. Index of the last variable in the slice plus 1.
      bk: array of mosek.boundkey. New bound keys.
      bl: array of double. New lower bounds.
      bu: array of double. New upper bounds.
    """
    if bk_ is not None and len(bk_) < ((last_) - (first_)):
      raise ValueError("Array argument bk is not long enough")
    if bk_ is None:
      raise ValueError("Argument bk cannot be None")
    if bk_ is None:
      raise ValueError("Argument bk may not be None")
    if bk_ is not None:
        _bk_tmp = (ctypes.c_int32 * len(bk_))(*bk_)
    else:
        _bk_tmp = None
    if bl_ is not None and len(bl_) < ((last_) - (first_)):
      raise ValueError("Array argument bl is not long enough")
    if bl_ is None:
      raise ValueError("Argument bl cannot be None")
    if bl_ is None:
      raise ValueError("Argument bl may not be None")
    if isinstance(bl_, numpy.ndarray) and bl_.dtype is numpy.dtype(numpy.float64) and bl_.flags.contiguous:
      _bl_copyarray = False
      _bl_tmp = ctypes.cast(bl_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bl_,array.ndarray) and bl_.dtype is numpy.dtype(array.float64) and  bl_.getsteplength() == 1:
      _bl_copyarray = False
      _bl_tmp = bl_.getdatacptr()
    elif bl_ is None:
      _bl_copyarray = False
      _bl_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bl_copyarray = True
      _bl_tmp = (ctypes.c_double * len(bl_))(*bl_)
    if bu_ is not None and len(bu_) < ((last_) - (first_)):
      raise ValueError("Array argument bu is not long enough")
    if bu_ is None:
      raise ValueError("Argument bu cannot be None")
    if bu_ is None:
      raise ValueError("Argument bu may not be None")
    if isinstance(bu_, numpy.ndarray) and bu_.dtype is numpy.dtype(numpy.float64) and bu_.flags.contiguous:
      _bu_copyarray = False
      _bu_tmp = ctypes.cast(bu_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(bu_,array.ndarray) and bu_.dtype is numpy.dtype(array.float64) and  bu_.getsteplength() == 1:
      _bu_copyarray = False
      _bu_tmp = bu_.getdatacptr()
    elif bu_ is None:
      _bu_copyarray = False
      _bu_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _bu_copyarray = True
      _bu_tmp = (ctypes.c_double * len(bu_))(*bu_)
    res = __library__.MSK_XX_putvarboundslice(self.__nativep,first_,last_,_bk_tmp,_bl_tmp,_bu_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_double)
  def putcfix(self,cfix_):
    """
    Replaces the fixed term in the objective.
  
    putcfix(self,cfix_)
      cfix: double. Fixed term in the objective.
    """
    res = __library__.MSK_XX_putcfix(self.__nativep,cfix_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_double)
  def putcj(self,j_,cj_):
    """
    Modifies one linear coefficient in the objective.
  
    putcj(self,j_,cj_)
      j: int. Index of the variable whose objective coefficient should be changed.
      cj: double. New coefficient value.
    """
    res = __library__.MSK_XX_putcj(self.__nativep,j_,cj_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(objsense))
  def putobjsense(self,sense_):
    """
    Sets the objective sense.
  
    putobjsense(self,sense_)
      sense: mosek.objsense. The objective sense of the task
    """
    res = __library__.MSK_XX_putobjsense(self.__nativep,sense_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getobjsense(self):
    """
    Gets the objective sense.
  
    getobjsense(self)
    returns: sense
      sense: mosek.objsense. The returned objective sense.
    """
    sense_ = ctypes.c_int32()
    res = __library__.MSK_XX_getobjsense(self.__nativep,ctypes.byref(sense_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _sense_return_value = objsense(sense_.value)
    return (_sense_return_value)
  @accepts(_accept_any,_make_intvector,_make_doublevector)
  def putclist(self,subj_,val_):
    """
    Modifies a part of the linear objective coefficients.
  
    putclist(self,subj_,val_)
      subj: array of int. Index of variables for which objective coefficients should be changed.
      val: array of double. New numerical values for the objective coefficients that should be modified.
    """
    num_ = None
    if num_ is None:
      num_ = len(subj_)
    else:
      num_ = min(num_,len(subj_))
    if num_ is None:
      num_ = len(val_)
    else:
      num_ = min(num_,len(val_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if val_ is None:
      raise ValueError("Argument val cannot be None")
    if val_ is None:
      raise ValueError("Argument val may not be None")
    if isinstance(val_, numpy.ndarray) and val_.dtype is numpy.dtype(numpy.float64) and val_.flags.contiguous:
      _val_copyarray = False
      _val_tmp = ctypes.cast(val_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(val_,array.ndarray) and val_.dtype is numpy.dtype(array.float64) and  val_.getsteplength() == 1:
      _val_copyarray = False
      _val_tmp = val_.getdatacptr()
    elif val_ is None:
      _val_copyarray = False
      _val_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _val_copyarray = True
      _val_tmp = (ctypes.c_double * len(val_))(*val_)
    res = __library__.MSK_XX_putclist(self.__nativep,num_,_subj_tmp,_val_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_doublevector)
  def putcslice(self,first_,last_,slice_):
    """
    Modifies a slice of the linear objective coefficients.
  
    putcslice(self,first_,last_,slice_)
      first: int. First element in the slice of c.
      last: int. Last element plus 1 of the slice in c to be changed.
      slice: array of double. New numerical values for the objective coefficients that should be modified.
    """
    if slice_ is not None and len(slice_) < ((last_) - (first_)):
      raise ValueError("Array argument slice is not long enough")
    if slice_ is None:
      raise ValueError("Argument slice cannot be None")
    if slice_ is None:
      raise ValueError("Argument slice may not be None")
    if isinstance(slice_, numpy.ndarray) and slice_.dtype is numpy.dtype(numpy.float64) and slice_.flags.contiguous:
      _slice_copyarray = False
      _slice_tmp = ctypes.cast(slice_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slice_,array.ndarray) and slice_.dtype is numpy.dtype(array.float64) and  slice_.getsteplength() == 1:
      _slice_copyarray = False
      _slice_tmp = slice_.getdatacptr()
    elif slice_ is None:
      _slice_copyarray = False
      _slice_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slice_copyarray = True
      _slice_tmp = (ctypes.c_double * len(slice_))(*slice_)
    res = __library__.MSK_XX_putcslice(self.__nativep,first_,last_,_slice_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_longvector,_make_doublevector)
  def putbarcj(self,j_,sub_,weights_):
    """
    Changes one element in barc.
  
    putbarcj(self,j_,sub_,weights_)
      j: int. Index of the element in barc$ that should be changed.
      sub: array of long. sub is list of indexes of those symmetric matrixes appearing in sum.
      weights: array of double. The weights of the terms in the weighted sum.
    """
    num_ = None
    if num_ is None:
      num_ = len(sub_)
    else:
      num_ = min(num_,len(sub_))
    if num_ is None:
      num_ = len(weights_)
    else:
      num_ = min(num_,len(weights_))
    if sub_ is None:
      raise ValueError("Argument sub cannot be None")
    if sub_ is None:
      raise ValueError("Argument sub may not be None")
    if isinstance(sub_, numpy.ndarray) and sub_.dtype is numpy.dtype(numpy.int64) and sub_.flags.contiguous:
      _sub_copyarray = False
      _sub_tmp = ctypes.cast(sub_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int64))
    elif isinstance(sub_,array.ndarray) and sub_.dtype is numpy.dtype(array.int64) and  sub_.getsteplength() == 1:
      _sub_copyarray = False
      _sub_tmp = sub_.getdatacptr()
    elif sub_ is None:
      _sub_copyarray = False
      _sub_tmp = ctypes.POINTER(ctypes.c_int64)()
    else:
      _sub_copyarray = True
      _sub_tmp = (ctypes.c_int64 * len(sub_))(*sub_)
    if weights_ is None:
      raise ValueError("Argument weights cannot be None")
    if weights_ is None:
      raise ValueError("Argument weights may not be None")
    if isinstance(weights_, numpy.ndarray) and weights_.dtype is numpy.dtype(numpy.float64) and weights_.flags.contiguous:
      _weights_copyarray = False
      _weights_tmp = ctypes.cast(weights_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(weights_,array.ndarray) and weights_.dtype is numpy.dtype(array.float64) and  weights_.getsteplength() == 1:
      _weights_copyarray = False
      _weights_tmp = weights_.getdatacptr()
    elif weights_ is None:
      _weights_copyarray = False
      _weights_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _weights_copyarray = True
      _weights_tmp = (ctypes.c_double * len(weights_))(*weights_)
    res = __library__.MSK_XX_putbarcj(self.__nativep,j_,num_,_sub_tmp,_weights_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_anyenum(conetype),_make_double,_make_intvector)
  def putcone(self,k_,conetype_,conepar_,submem_):
    """
    Replaces a conic constraint.
  
    putcone(self,k_,conetype_,conepar_,submem_)
      k: int. Index of the cone.
      conetype: mosek.conetype. Specifies the type of the cone.
      conepar: double. This argument is currently not used. Can be set to 0.0.
      submem: array of int. Variable subscripts of the members in the cone.
    """
    nummem_ = None
    if nummem_ is None:
      nummem_ = len(submem_)
    else:
      nummem_ = min(nummem_,len(submem_))
    if submem_ is None:
      raise ValueError("Argument submem cannot be None")
    if submem_ is None:
      raise ValueError("Argument submem may not be None")
    if isinstance(submem_, numpy.ndarray) and submem_.dtype is numpy.dtype(numpy.int32) and submem_.flags.contiguous:
      _submem_copyarray = False
      _submem_tmp = ctypes.cast(submem_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(submem_,array.ndarray) and submem_.dtype is numpy.dtype(array.int32) and  submem_.getsteplength() == 1:
      _submem_copyarray = False
      _submem_tmp = submem_.getdatacptr()
    elif submem_ is None:
      _submem_copyarray = False
      _submem_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _submem_copyarray = True
      _submem_tmp = (ctypes.c_int32 * len(submem_))(*submem_)
    res = __library__.MSK_XX_putcone(self.__nativep,k_,conetype_,conepar_,nummem_,_submem_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_intvector,_make_intvector,_make_doublevector)
  def appendsparsesymmat(self,dim_,subi_,subj_,valij_):
    """
    Appends a general sparse symmetric matrix to the vector E of symmetric matrixes.
  
    appendsparsesymmat(self,dim_,subi_,subj_,valij_)
      dim: int. Dimension of the symmetric matrix that is appended.
      subi: array of int. Row subscript in the triplets.
      subj: array of int. Column subscripts in the triplets.
      valij: array of double. Values of each triplet.
    returns: idx
      idx: long. Unique index assigned to inputted matrix.
    """
    nz_ = None
    if nz_ is None:
      nz_ = len(subi_)
    else:
      nz_ = min(nz_,len(subi_))
    if nz_ is None:
      nz_ = len(subj_)
    else:
      nz_ = min(nz_,len(subj_))
    if nz_ is None:
      nz_ = len(valij_)
    else:
      nz_ = min(nz_,len(valij_))
    if subi_ is None:
      raise ValueError("Argument subi cannot be None")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if valij_ is None:
      raise ValueError("Argument valij cannot be None")
    if valij_ is None:
      raise ValueError("Argument valij may not be None")
    if isinstance(valij_, numpy.ndarray) and valij_.dtype is numpy.dtype(numpy.float64) and valij_.flags.contiguous:
      _valij_copyarray = False
      _valij_tmp = ctypes.cast(valij_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valij_,array.ndarray) and valij_.dtype is numpy.dtype(array.float64) and  valij_.getsteplength() == 1:
      _valij_copyarray = False
      _valij_tmp = valij_.getdatacptr()
    elif valij_ is None:
      _valij_copyarray = False
      _valij_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valij_copyarray = True
      _valij_tmp = (ctypes.c_double * len(valij_))(*valij_)
    idx_ = ctypes.c_int64()
    idx_ = ctypes.c_int64()
    res = __library__.MSK_XX_appendsparsesymmat(self.__nativep,dim_,nz_,_subi_tmp,_subj_tmp,_valij_tmp,ctypes.byref(idx_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _idx_return_value = idx_.value
    return (_idx_return_value)
  @accepts(_accept_any,_make_long)
  def getsymmatinfo(self,idx_):
    """
    Obtains information of  a matrix from the symmetric matrix storage E.
  
    getsymmatinfo(self,idx_)
      idx: long. Index of the matrix that is requested information about.
    returns: dim,nz,type
      dim: int. Returns the dimension of the requested matrix.
      nz: long. Returns the number of non-zeros in the requested matrix.
      type: mosek.symmattype. Returns the type of the requested matrix.
    """
    dim_ = ctypes.c_int32()
    dim_ = ctypes.c_int32()
    nz_ = ctypes.c_int64()
    nz_ = ctypes.c_int64()
    type_ = ctypes.c_int32()
    res = __library__.MSK_XX_getsymmatinfo(self.__nativep,idx_,ctypes.byref(dim_),ctypes.byref(nz_),ctypes.byref(type_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _dim_return_value = dim_.value
    _nz_return_value = nz_.value
    _type_return_value = symmattype(type_.value)
    return (_dim_return_value,_nz_return_value,_type_return_value)
  @accepts(_accept_any)
  def getnumsymmat(self):
    """
    Get the number of symmetric matrixes stored.
  
    getnumsymmat(self)
    returns: num
      num: long. Returns the number of symmetric sparse matrixes.
    """
    num_ = ctypes.c_int64()
    num_ = ctypes.c_int64()
    res = __library__.MSK_XX_getnumsymmat(self.__nativep,ctypes.byref(num_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _num_return_value = num_.value
    return (_num_return_value)
  @accepts(_accept_any,_make_long,_accept_intvector,_accept_intvector,_accept_doublevector)
  def getsparsesymmat(self,idx_,subi_,subj_,valij_):
    """
    Gets a single symmetric matrix from the matrix store.
  
    getsparsesymmat(self,idx_,subi_,subj_,valij_)
      idx: long. Index of the matrix to get.
      subi: array of int. Row subscripts of the matrix non-zero elements.
      subj: array of int. Column subscripts of the matrix non-zero elements.
      valij: array of double. Coefficients of the matrix non-zero elements.
    """
    maxlen_ = self.getsymmatinfo((idx_))[1]
    if subi_ is not None and len(subi_) < (maxlen_):
      raise ValueError("Array argument subi is not long enough")
    if isinstance(subi_,numpy.ndarray) and not subi_.flags.writeable:
      raise ValueError("Argument subi must be writable")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if subj_ is not None and len(subj_) < (maxlen_):
      raise ValueError("Array argument subj is not long enough")
    if isinstance(subj_,numpy.ndarray) and not subj_.flags.writeable:
      raise ValueError("Argument subj must be writable")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if valij_ is not None and len(valij_) < (maxlen_):
      raise ValueError("Array argument valij is not long enough")
    if isinstance(valij_,numpy.ndarray) and not valij_.flags.writeable:
      raise ValueError("Argument valij must be writable")
    if isinstance(valij_, numpy.ndarray) and valij_.dtype is numpy.dtype(numpy.float64) and valij_.flags.contiguous:
      _valij_copyarray = False
      _valij_tmp = ctypes.cast(valij_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(valij_,array.ndarray) and valij_.dtype is numpy.dtype(array.float64) and  valij_.getsteplength() == 1:
      _valij_copyarray = False
      _valij_tmp = valij_.getdatacptr()
    elif valij_ is None:
      _valij_copyarray = False
      _valij_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _valij_copyarray = True
      _valij_tmp = (ctypes.c_double * len(valij_))(*valij_)
    res = __library__.MSK_XX_getsparsesymmat(self.__nativep,idx_,maxlen_,_subi_tmp,_subj_tmp,_valij_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _subi_copyarray:
      if isinstance(subi_, numpy.ndarray) and False:
        subi_[:] = list(_subi_tmp)
      else:
        subi_[:] = _subi_tmp
    if _subj_copyarray:
      if isinstance(subj_, numpy.ndarray) and False:
        subj_[:] = list(_subj_tmp)
      else:
        subj_[:] = _subj_tmp
    if _valij_copyarray:
      if isinstance(valij_, numpy.ndarray) and False:
        valij_[:] = list(_valij_tmp)
      else:
        valij_[:] = _valij_tmp
  @accepts(_accept_any,_accept_anyenum(dparam),_make_double)
  def putdouparam(self,param_,parvalue_):
    """
    Sets a double parameter.
  
    putdouparam(self,param_,parvalue_)
      param: mosek.dparam. Which parameter.
      parvalue: double. Parameter value.
    """
    res = __library__.MSK_XX_putdouparam(self.__nativep,param_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(iparam),_make_int)
  def putintparam(self,param_,parvalue_):
    """
    Sets an integer parameter.
  
    putintparam(self,param_,parvalue_)
      param: mosek.iparam. Which parameter.
      parvalue: int. Parameter value.
    """
    res = __library__.MSK_XX_putintparam(self.__nativep,param_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def putmaxnumcon(self,maxnumcon_):
    """
    Sets the number of preallocated constraints in the optimization task.
  
    putmaxnumcon(self,maxnumcon_)
      maxnumcon: int. Number of preallocated constraints in the optimization task.
    """
    res = __library__.MSK_XX_putmaxnumcon(self.__nativep,maxnumcon_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def putmaxnumcone(self,maxnumcone_):
    """
    Sets the number of preallocated conic constraints in the optimization task.
  
    putmaxnumcone(self,maxnumcone_)
      maxnumcone: int. Number of preallocated conic constraints in the optimization task.
    """
    res = __library__.MSK_XX_putmaxnumcone(self.__nativep,maxnumcone_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getmaxnumcone(self):
    """
    Obtains the number of preallocated cones in the optimization task.
  
    getmaxnumcone(self)
    returns: maxnumcone
      maxnumcone: int. Number of preallocated conic constraints in the optimization task.
    """
    maxnumcone_ = ctypes.c_int32()
    maxnumcone_ = ctypes.c_int32()
    res = __library__.MSK_XX_getmaxnumcone(self.__nativep,ctypes.byref(maxnumcone_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumcone_return_value = maxnumcone_.value
    return (_maxnumcone_return_value)
  @accepts(_accept_any,_make_int)
  def putmaxnumvar(self,maxnumvar_):
    """
    Sets the number of preallocated variables in the optimization task.
  
    putmaxnumvar(self,maxnumvar_)
      maxnumvar: int. Number of preallocated variables in the optimization task.
    """
    res = __library__.MSK_XX_putmaxnumvar(self.__nativep,maxnumvar_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def putmaxnumbarvar(self,maxnumbarvar_):
    """
    Sets the number of preallocated symmetric matrix variables in the optimization task.
  
    putmaxnumbarvar(self,maxnumbarvar_)
      maxnumbarvar: int. The maximum number of semidefinite variables.
    """
    res = __library__.MSK_XX_putmaxnumbarvar(self.__nativep,maxnumbarvar_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_long)
  def putmaxnumanz(self,maxnumanz_):
    """
    The function changes the size of the preallocated storage for linear coefficients.
  
    putmaxnumanz(self,maxnumanz_)
      maxnumanz: long. New size of the storage reserved for storing the linear coefficient matrix.
    """
    res = __library__.MSK_XX_putmaxnumanz(self.__nativep,maxnumanz_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_long)
  def putmaxnumqnz(self,maxnumqnz_):
    """
    Changes the size of the preallocated storage for quadratic terms.
  
    putmaxnumqnz(self,maxnumqnz_)
      maxnumqnz: long. Number of non-zero elements preallocated in quadratic coefficient matrixes.
    """
    res = __library__.MSK_XX_putmaxnumqnz(self.__nativep,maxnumqnz_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getmaxnumqnz(self):
    """
    Obtains the number of preallocated non-zeros for all quadratic terms in objective and constraints.
  
    getmaxnumqnz(self)
    returns: maxnumqnz
      maxnumqnz: long. Number of non-zero elements preallocated in quadratic coefficient matrixes.
    """
    maxnumqnz_ = ctypes.c_int64()
    maxnumqnz_ = ctypes.c_int64()
    res = __library__.MSK_XX_getmaxnumqnz64(self.__nativep,ctypes.byref(maxnumqnz_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _maxnumqnz_return_value = maxnumqnz_.value
    return (_maxnumqnz_return_value)
  @accepts(_accept_any,_accept_str,_make_double)
  def putnadouparam(self,paramname_,parvalue_):
    """
    Sets a double parameter.
  
    putnadouparam(self,paramname_,parvalue_)
      paramname: str|unicode. Name of a parameter.
      parvalue: double. Parameter value.
    """
    if isinstance(paramname_,unicode):
      paramname_ = paramname_.encode("utf-8")
    res = __library__.MSK_XX_putnadouparam(self.__nativep,paramname_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str,_make_int)
  def putnaintparam(self,paramname_,parvalue_):
    """
    Sets an integer parameter.
  
    putnaintparam(self,paramname_,parvalue_)
      paramname: str|unicode. Name of a parameter.
      parvalue: int. Parameter value.
    """
    if isinstance(paramname_,unicode):
      paramname_ = paramname_.encode("utf-8")
    res = __library__.MSK_XX_putnaintparam(self.__nativep,paramname_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str,_accept_str)
  def putnastrparam(self,paramname_,parvalue_):
    """
    Sets a string parameter.
  
    putnastrparam(self,paramname_,parvalue_)
      paramname: str|unicode. Name of a parameter.
      parvalue: str|unicode. Parameter value.
    """
    if isinstance(paramname_,unicode):
      paramname_ = paramname_.encode("utf-8")
    if isinstance(parvalue_,unicode):
      parvalue_ = parvalue_.encode("utf-8")
    res = __library__.MSK_XX_putnastrparam(self.__nativep,paramname_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def putobjname(self,objname_):
    """
    Assigns a new name to the objective.
  
    putobjname(self,objname_)
      objname: str|unicode. Name of the objective.
    """
    if isinstance(objname_,unicode):
      objname_ = objname_.encode("utf-8")
    res = __library__.MSK_XX_putobjname(self.__nativep,objname_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str,_accept_str)
  def putparam(self,parname_,parvalue_):
    """
    Modifies the value of parameter.
  
    putparam(self,parname_,parvalue_)
      parname: str|unicode. Parameter name.
      parvalue: str|unicode. Parameter value.
    """
    if isinstance(parname_,unicode):
      parname_ = parname_.encode("utf-8")
    if isinstance(parvalue_,unicode):
      parvalue_ = parvalue_.encode("utf-8")
    res = __library__.MSK_XX_putparam(self.__nativep,parname_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_intvector,_make_intvector,_make_doublevector)
  def putqcon(self,qcsubk_,qcsubi_,qcsubj_,qcval_):
    """
    Replaces all quadratic terms in constraints.
  
    putqcon(self,qcsubk_,qcsubi_,qcsubj_,qcval_)
      qcsubk: array of int. Constraint subscripts for quadratic coefficients.
      qcsubi: array of int. Row subscripts for quadratic constraint matrix.
      qcsubj: array of int. Column subscripts for quadratic constraint matrix.
      qcval: array of double. Quadratic constraint coefficient values.
    """
    numqcnz_ = None
    if numqcnz_ is None:
      numqcnz_ = len(qcsubi_)
    else:
      numqcnz_ = min(numqcnz_,len(qcsubi_))
    if numqcnz_ is None:
      numqcnz_ = len(qcsubj_)
    else:
      numqcnz_ = min(numqcnz_,len(qcsubj_))
    if numqcnz_ is None:
      numqcnz_ = len(qcval_)
    else:
      numqcnz_ = min(numqcnz_,len(qcval_))
    if qcsubk_ is None:
      raise ValueError("Argument qcsubk cannot be None")
    if qcsubk_ is None:
      raise ValueError("Argument qcsubk may not be None")
    if isinstance(qcsubk_, numpy.ndarray) and qcsubk_.dtype is numpy.dtype(numpy.int32) and qcsubk_.flags.contiguous:
      _qcsubk_copyarray = False
      _qcsubk_tmp = ctypes.cast(qcsubk_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubk_,array.ndarray) and qcsubk_.dtype is numpy.dtype(array.int32) and  qcsubk_.getsteplength() == 1:
      _qcsubk_copyarray = False
      _qcsubk_tmp = qcsubk_.getdatacptr()
    elif qcsubk_ is None:
      _qcsubk_copyarray = False
      _qcsubk_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubk_copyarray = True
      _qcsubk_tmp = (ctypes.c_int32 * len(qcsubk_))(*qcsubk_)
    if qcsubi_ is None:
      raise ValueError("Argument qcsubi cannot be None")
    if qcsubi_ is None:
      raise ValueError("Argument qcsubi may not be None")
    if isinstance(qcsubi_, numpy.ndarray) and qcsubi_.dtype is numpy.dtype(numpy.int32) and qcsubi_.flags.contiguous:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.cast(qcsubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubi_,array.ndarray) and qcsubi_.dtype is numpy.dtype(array.int32) and  qcsubi_.getsteplength() == 1:
      _qcsubi_copyarray = False
      _qcsubi_tmp = qcsubi_.getdatacptr()
    elif qcsubi_ is None:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubi_copyarray = True
      _qcsubi_tmp = (ctypes.c_int32 * len(qcsubi_))(*qcsubi_)
    if qcsubj_ is None:
      raise ValueError("Argument qcsubj cannot be None")
    if qcsubj_ is None:
      raise ValueError("Argument qcsubj may not be None")
    if isinstance(qcsubj_, numpy.ndarray) and qcsubj_.dtype is numpy.dtype(numpy.int32) and qcsubj_.flags.contiguous:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.cast(qcsubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubj_,array.ndarray) and qcsubj_.dtype is numpy.dtype(array.int32) and  qcsubj_.getsteplength() == 1:
      _qcsubj_copyarray = False
      _qcsubj_tmp = qcsubj_.getdatacptr()
    elif qcsubj_ is None:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubj_copyarray = True
      _qcsubj_tmp = (ctypes.c_int32 * len(qcsubj_))(*qcsubj_)
    if qcval_ is None:
      raise ValueError("Argument qcval cannot be None")
    if qcval_ is None:
      raise ValueError("Argument qcval may not be None")
    if isinstance(qcval_, numpy.ndarray) and qcval_.dtype is numpy.dtype(numpy.float64) and qcval_.flags.contiguous:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.cast(qcval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qcval_,array.ndarray) and qcval_.dtype is numpy.dtype(array.float64) and  qcval_.getsteplength() == 1:
      _qcval_copyarray = False
      _qcval_tmp = qcval_.getdatacptr()
    elif qcval_ is None:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qcval_copyarray = True
      _qcval_tmp = (ctypes.c_double * len(qcval_))(*qcval_)
    res = __library__.MSK_XX_putqcon(self.__nativep,numqcnz_,_qcsubk_tmp,_qcsubi_tmp,_qcsubj_tmp,_qcval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_intvector,_make_intvector,_make_doublevector)
  def putqconk(self,k_,qcsubi_,qcsubj_,qcval_):
    """
    Replaces all quadratic terms in a single constraint.
  
    putqconk(self,k_,qcsubi_,qcsubj_,qcval_)
      k: int. The constraint in which the new quadratic elements are inserted.
      qcsubi: array of int. Row subscripts for quadratic constraint matrix.
      qcsubj: array of int. Column subscripts for quadratic constraint matrix.
      qcval: array of double. Quadratic constraint coefficient values.
    """
    numqcnz_ = None
    if numqcnz_ is None:
      numqcnz_ = len(qcsubi_)
    else:
      numqcnz_ = min(numqcnz_,len(qcsubi_))
    if numqcnz_ is None:
      numqcnz_ = len(qcsubj_)
    else:
      numqcnz_ = min(numqcnz_,len(qcsubj_))
    if numqcnz_ is None:
      numqcnz_ = len(qcval_)
    else:
      numqcnz_ = min(numqcnz_,len(qcval_))
    if qcsubi_ is None:
      raise ValueError("Argument qcsubi cannot be None")
    if qcsubi_ is None:
      raise ValueError("Argument qcsubi may not be None")
    if isinstance(qcsubi_, numpy.ndarray) and qcsubi_.dtype is numpy.dtype(numpy.int32) and qcsubi_.flags.contiguous:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.cast(qcsubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubi_,array.ndarray) and qcsubi_.dtype is numpy.dtype(array.int32) and  qcsubi_.getsteplength() == 1:
      _qcsubi_copyarray = False
      _qcsubi_tmp = qcsubi_.getdatacptr()
    elif qcsubi_ is None:
      _qcsubi_copyarray = False
      _qcsubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubi_copyarray = True
      _qcsubi_tmp = (ctypes.c_int32 * len(qcsubi_))(*qcsubi_)
    if qcsubj_ is None:
      raise ValueError("Argument qcsubj cannot be None")
    if qcsubj_ is None:
      raise ValueError("Argument qcsubj may not be None")
    if isinstance(qcsubj_, numpy.ndarray) and qcsubj_.dtype is numpy.dtype(numpy.int32) and qcsubj_.flags.contiguous:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.cast(qcsubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qcsubj_,array.ndarray) and qcsubj_.dtype is numpy.dtype(array.int32) and  qcsubj_.getsteplength() == 1:
      _qcsubj_copyarray = False
      _qcsubj_tmp = qcsubj_.getdatacptr()
    elif qcsubj_ is None:
      _qcsubj_copyarray = False
      _qcsubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qcsubj_copyarray = True
      _qcsubj_tmp = (ctypes.c_int32 * len(qcsubj_))(*qcsubj_)
    if qcval_ is None:
      raise ValueError("Argument qcval cannot be None")
    if qcval_ is None:
      raise ValueError("Argument qcval may not be None")
    if isinstance(qcval_, numpy.ndarray) and qcval_.dtype is numpy.dtype(numpy.float64) and qcval_.flags.contiguous:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.cast(qcval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qcval_,array.ndarray) and qcval_.dtype is numpy.dtype(array.float64) and  qcval_.getsteplength() == 1:
      _qcval_copyarray = False
      _qcval_tmp = qcval_.getdatacptr()
    elif qcval_ is None:
      _qcval_copyarray = False
      _qcval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qcval_copyarray = True
      _qcval_tmp = (ctypes.c_double * len(qcval_))(*qcval_)
    res = __library__.MSK_XX_putqconk(self.__nativep,k_,numqcnz_,_qcsubi_tmp,_qcsubj_tmp,_qcval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_intvector,_make_doublevector)
  def putqobj(self,qosubi_,qosubj_,qoval_):
    """
    Replaces all quadratic terms in the objective.
  
    putqobj(self,qosubi_,qosubj_,qoval_)
      qosubi: array of int. Row subscripts for quadratic objective coefficients.
      qosubj: array of int. Column subscripts for quadratic objective coefficients.
      qoval: array of double. Quadratic objective coefficient values.
    """
    numqonz_ = None
    if numqonz_ is None:
      numqonz_ = len(qosubi_)
    else:
      numqonz_ = min(numqonz_,len(qosubi_))
    if numqonz_ is None:
      numqonz_ = len(qosubj_)
    else:
      numqonz_ = min(numqonz_,len(qosubj_))
    if numqonz_ is None:
      numqonz_ = len(qoval_)
    else:
      numqonz_ = min(numqonz_,len(qoval_))
    if qosubi_ is None:
      raise ValueError("Argument qosubi cannot be None")
    if qosubi_ is None:
      raise ValueError("Argument qosubi may not be None")
    if isinstance(qosubi_, numpy.ndarray) and qosubi_.dtype is numpy.dtype(numpy.int32) and qosubi_.flags.contiguous:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.cast(qosubi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubi_,array.ndarray) and qosubi_.dtype is numpy.dtype(array.int32) and  qosubi_.getsteplength() == 1:
      _qosubi_copyarray = False
      _qosubi_tmp = qosubi_.getdatacptr()
    elif qosubi_ is None:
      _qosubi_copyarray = False
      _qosubi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubi_copyarray = True
      _qosubi_tmp = (ctypes.c_int32 * len(qosubi_))(*qosubi_)
    if qosubj_ is None:
      raise ValueError("Argument qosubj cannot be None")
    if qosubj_ is None:
      raise ValueError("Argument qosubj may not be None")
    if isinstance(qosubj_, numpy.ndarray) and qosubj_.dtype is numpy.dtype(numpy.int32) and qosubj_.flags.contiguous:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.cast(qosubj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(qosubj_,array.ndarray) and qosubj_.dtype is numpy.dtype(array.int32) and  qosubj_.getsteplength() == 1:
      _qosubj_copyarray = False
      _qosubj_tmp = qosubj_.getdatacptr()
    elif qosubj_ is None:
      _qosubj_copyarray = False
      _qosubj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _qosubj_copyarray = True
      _qosubj_tmp = (ctypes.c_int32 * len(qosubj_))(*qosubj_)
    if qoval_ is None:
      raise ValueError("Argument qoval cannot be None")
    if qoval_ is None:
      raise ValueError("Argument qoval may not be None")
    if isinstance(qoval_, numpy.ndarray) and qoval_.dtype is numpy.dtype(numpy.float64) and qoval_.flags.contiguous:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.cast(qoval_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(qoval_,array.ndarray) and qoval_.dtype is numpy.dtype(array.float64) and  qoval_.getsteplength() == 1:
      _qoval_copyarray = False
      _qoval_tmp = qoval_.getdatacptr()
    elif qoval_ is None:
      _qoval_copyarray = False
      _qoval_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _qoval_copyarray = True
      _qoval_tmp = (ctypes.c_double * len(qoval_))(*qoval_)
    res = __library__.MSK_XX_putqobj(self.__nativep,numqonz_,_qosubi_tmp,_qosubj_tmp,_qoval_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_double)
  def putqobjij(self,i_,j_,qoij_):
    """
    Replaces one coefficient in the quadratic term in                     the objective.
  
    putqobjij(self,i_,j_,qoij_)
      i: int. Row index for the coefficient to be replaced.
      j: int. Column index for the coefficient to be replaced.
      qoij: double. The new coefficient value.
    """
    res = __library__.MSK_XX_putqobjij(self.__nativep,i_,j_,qoij_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_make_anyenumvector(stakey),_make_anyenumvector(stakey),_make_anyenumvector(stakey),_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector,_make_doublevector)
  def putsolution(self,whichsol_,skc_,skx_,skn_,xc_,xx_,y_,slc_,suc_,slx_,sux_,snx_):
    """
    Inserts a solution.
  
    putsolution(self,whichsol_,skc_,skx_,skn_,xc_,xx_,y_,slc_,suc_,slx_,sux_,snx_)
      whichsol: mosek.soltype. Selects a solution.
      skc: array of mosek.stakey. Status keys for the constraints.
      skx: array of mosek.stakey. Status keys for the variables.
      skn: array of mosek.stakey. Status keys for the conic constraints.
      xc: array of double. Primal constraint solution.
      xx: array of double. Primal variable solution.
      y: array of double. Vector of dual variables corresponding to the constraints.
      slc: array of double. Dual variables corresponding to the lower bounds on the constraints.
      suc: array of double. Dual variables corresponding to the upper bounds on the constraints.
      slx: array of double. Dual variables corresponding to the lower bounds on the variables.
      sux: array of double. Dual variables corresponding to the upper bounds on the variables.
      snx: array of double. Dual variables corresponding to the conic constraints on the variables.
    """
    if skc_ is not None:
        _skc_tmp = (ctypes.c_int32 * len(skc_))(*skc_)
    else:
        _skc_tmp = None
    if skx_ is not None:
        _skx_tmp = (ctypes.c_int32 * len(skx_))(*skx_)
    else:
        _skx_tmp = None
    if skn_ is not None:
        _skn_tmp = (ctypes.c_int32 * len(skn_))(*skn_)
    else:
        _skn_tmp = None
    if isinstance(xc_, numpy.ndarray) and xc_.dtype is numpy.dtype(numpy.float64) and xc_.flags.contiguous:
      _xc_copyarray = False
      _xc_tmp = ctypes.cast(xc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xc_,array.ndarray) and xc_.dtype is numpy.dtype(array.float64) and  xc_.getsteplength() == 1:
      _xc_copyarray = False
      _xc_tmp = xc_.getdatacptr()
    elif xc_ is None:
      _xc_copyarray = False
      _xc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xc_copyarray = True
      _xc_tmp = (ctypes.c_double * len(xc_))(*xc_)
    if isinstance(xx_, numpy.ndarray) and xx_.dtype is numpy.dtype(numpy.float64) and xx_.flags.contiguous:
      _xx_copyarray = False
      _xx_tmp = ctypes.cast(xx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(xx_,array.ndarray) and xx_.dtype is numpy.dtype(array.float64) and  xx_.getsteplength() == 1:
      _xx_copyarray = False
      _xx_tmp = xx_.getdatacptr()
    elif xx_ is None:
      _xx_copyarray = False
      _xx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _xx_copyarray = True
      _xx_tmp = (ctypes.c_double * len(xx_))(*xx_)
    if isinstance(y_, numpy.ndarray) and y_.dtype is numpy.dtype(numpy.float64) and y_.flags.contiguous:
      _y_copyarray = False
      _y_tmp = ctypes.cast(y_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(y_,array.ndarray) and y_.dtype is numpy.dtype(array.float64) and  y_.getsteplength() == 1:
      _y_copyarray = False
      _y_tmp = y_.getdatacptr()
    elif y_ is None:
      _y_copyarray = False
      _y_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _y_copyarray = True
      _y_tmp = (ctypes.c_double * len(y_))(*y_)
    if isinstance(slc_, numpy.ndarray) and slc_.dtype is numpy.dtype(numpy.float64) and slc_.flags.contiguous:
      _slc_copyarray = False
      _slc_tmp = ctypes.cast(slc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slc_,array.ndarray) and slc_.dtype is numpy.dtype(array.float64) and  slc_.getsteplength() == 1:
      _slc_copyarray = False
      _slc_tmp = slc_.getdatacptr()
    elif slc_ is None:
      _slc_copyarray = False
      _slc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slc_copyarray = True
      _slc_tmp = (ctypes.c_double * len(slc_))(*slc_)
    if isinstance(suc_, numpy.ndarray) and suc_.dtype is numpy.dtype(numpy.float64) and suc_.flags.contiguous:
      _suc_copyarray = False
      _suc_tmp = ctypes.cast(suc_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(suc_,array.ndarray) and suc_.dtype is numpy.dtype(array.float64) and  suc_.getsteplength() == 1:
      _suc_copyarray = False
      _suc_tmp = suc_.getdatacptr()
    elif suc_ is None:
      _suc_copyarray = False
      _suc_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _suc_copyarray = True
      _suc_tmp = (ctypes.c_double * len(suc_))(*suc_)
    if isinstance(slx_, numpy.ndarray) and slx_.dtype is numpy.dtype(numpy.float64) and slx_.flags.contiguous:
      _slx_copyarray = False
      _slx_tmp = ctypes.cast(slx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(slx_,array.ndarray) and slx_.dtype is numpy.dtype(array.float64) and  slx_.getsteplength() == 1:
      _slx_copyarray = False
      _slx_tmp = slx_.getdatacptr()
    elif slx_ is None:
      _slx_copyarray = False
      _slx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _slx_copyarray = True
      _slx_tmp = (ctypes.c_double * len(slx_))(*slx_)
    if isinstance(sux_, numpy.ndarray) and sux_.dtype is numpy.dtype(numpy.float64) and sux_.flags.contiguous:
      _sux_copyarray = False
      _sux_tmp = ctypes.cast(sux_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(sux_,array.ndarray) and sux_.dtype is numpy.dtype(array.float64) and  sux_.getsteplength() == 1:
      _sux_copyarray = False
      _sux_tmp = sux_.getdatacptr()
    elif sux_ is None:
      _sux_copyarray = False
      _sux_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _sux_copyarray = True
      _sux_tmp = (ctypes.c_double * len(sux_))(*sux_)
    if isinstance(snx_, numpy.ndarray) and snx_.dtype is numpy.dtype(numpy.float64) and snx_.flags.contiguous:
      _snx_copyarray = False
      _snx_tmp = ctypes.cast(snx_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(snx_,array.ndarray) and snx_.dtype is numpy.dtype(array.float64) and  snx_.getsteplength() == 1:
      _snx_copyarray = False
      _snx_tmp = snx_.getdatacptr()
    elif snx_ is None:
      _snx_copyarray = False
      _snx_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _snx_copyarray = True
      _snx_tmp = (ctypes.c_double * len(snx_))(*snx_)
    res = __library__.MSK_XX_putsolution(self.__nativep,whichsol_,_skc_tmp,_skx_tmp,_skn_tmp,_xc_tmp,_xx_tmp,_y_tmp,_slc_tmp,_suc_tmp,_slx_tmp,_sux_tmp,_snx_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(accmode),_make_int,_accept_anyenum(soltype),_accept_anyenum(stakey),_make_double,_make_double,_make_double,_make_double)
  def putsolutioni(self,accmode_,i_,whichsol_,sk_,x_,sl_,su_,sn_):
    """
    Sets the primal and dual solution information for a single constraint or variable.
  
    putsolutioni(self,accmode_,i_,whichsol_,sk_,x_,sl_,su_,sn_)
      accmode: mosek.accmode. Defines whether solution information for a constraint or for a variable is modified.
      i: int. Index of the constraint or variable.
      whichsol: mosek.soltype. Selects a solution.
      sk: mosek.stakey. Status key of the constraint or variable.
      x: double. Solution value of the primal constraint or variable.
      sl: double. Solution value of the dual variable associated with the lower bound.
      su: double. Solution value of the dual variable associated with the upper bound.
      sn: double. Solution value of the dual variable associated with the cone constraint.
    """
    res = __library__.MSK_XX_putsolutioni(self.__nativep,accmode_,i_,whichsol_,sk_,x_,sl_,su_,sn_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_anyenum(soltype),_make_double)
  def putsolutionyi(self,i_,whichsol_,y_):
    """
    Inputs the dual variable of a solution.
  
    putsolutionyi(self,i_,whichsol_,y_)
      i: int. Index of the dual variable.
      whichsol: mosek.soltype. Selects a solution.
      y: double. Solution value of the dual variable.
    """
    res = __library__.MSK_XX_putsolutionyi(self.__nativep,i_,whichsol_,y_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(sparam),_accept_str)
  def putstrparam(self,param_,parvalue_):
    """
    Sets a string parameter.
  
    putstrparam(self,param_,parvalue_)
      param: mosek.sparam. Which parameter.
      parvalue: str|unicode. Parameter value.
    """
    if isinstance(parvalue_,unicode):
      parvalue_ = parvalue_.encode("utf-8")
    res = __library__.MSK_XX_putstrparam(self.__nativep,param_,parvalue_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def puttaskname(self,taskname_):
    """
    Assigns a new name to the task.
  
    puttaskname(self,taskname_)
      taskname: str|unicode. Name assigned to the task.
    """
    if isinstance(taskname_,unicode):
      taskname_ = taskname_.encode("utf-8")
    res = __library__.MSK_XX_puttaskname(self.__nativep,taskname_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_accept_anyenum(variabletype))
  def putvartype(self,j_,vartype_):
    """
    Sets the variable type of one variable.
  
    putvartype(self,j_,vartype_)
      j: int. Index of the variable.
      vartype: mosek.variabletype. The new variable type.
    """
    res = __library__.MSK_XX_putvartype(self.__nativep,j_,vartype_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_anyenumvector(variabletype))
  def putvartypelist(self,subj_,vartype_):
    """
    Sets the variable type for one or more variables.
  
    putvartypelist(self,subj_,vartype_)
      subj: array of int. A list of variable indexes for which the variable                            type should be changed.
      vartype: array of mosek.variabletype. A list of variable types.
    """
    num_ = None
    if num_ is None:
      num_ = len(subj_)
    else:
      num_ = min(num_,len(subj_))
    if num_ is None:
      num_ = len(vartype_)
    else:
      num_ = min(num_,len(vartype_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if vartype_ is None:
      raise ValueError("Argument vartype cannot be None")
    if vartype_ is None:
      raise ValueError("Argument vartype may not be None")
    if vartype_ is not None:
        _vartype_tmp = (ctypes.c_int32 * len(vartype_))(*vartype_)
    else:
        _vartype_tmp = None
    res = __library__.MSK_XX_putvartypelist(self.__nativep,num_,_subj_tmp,_vartype_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_accept_anyenum(branchdir))
  def putvarbranchorder(self,j_,priority_,direction_):
    """
    Assigns a branching priority and direction to a variable.
  
    putvarbranchorder(self,j_,priority_,direction_)
      j: int. Index of the variable.
      priority: int. The branching priority that should be assigned to the j'th variable.
      direction: mosek.branchdir. Specifies the preferred branching direction for the j'th variable.
    """
    res = __library__.MSK_XX_putvarbranchorder(self.__nativep,j_,priority_,direction_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int)
  def getvarbranchpri(self,j_):
    """
    Obtains the branching priority for a variable.
  
    getvarbranchpri(self,j_)
      j: int. Index of the variable.
    returns: priority
      priority: int. The branching priority assigned to the j'th variable.
    """
    priority_ = ctypes.c_int32()
    priority_ = ctypes.c_int32()
    res = __library__.MSK_XX_getvarbranchpri(self.__nativep,j_,ctypes.byref(priority_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _priority_return_value = priority_.value
    return (_priority_return_value)
  @accepts(_accept_any,_make_int)
  def getvarbranchdir(self,j_):
    """
    Obtains the branching direction for a variable.
  
    getvarbranchdir(self,j_)
      j: int. Index of the variable.
    returns: direction
      direction: mosek.branchdir. The branching direction assigned to the j'th variable.
    """
    direction_ = ctypes.c_int32()
    res = __library__.MSK_XX_getvarbranchdir(self.__nativep,j_,ctypes.byref(direction_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _direction_return_value = branchdir(direction_.value)
    return (_direction_return_value)
  @accepts(_accept_any,_accept_str,_accept_anyenum(dataformat),_accept_anyenum(compresstype))
  def readdataformat(self,filename_,format_,compress_):
    """
    Reads problem data from a file.
  
    readdataformat(self,filename_,format_,compress_)
      filename: str|unicode. Input data file name.
      format: mosek.dataformat. File data format.
      compress: mosek.compresstype. File compression type.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_readdataformat(self.__nativep,filename_,format_,compress_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def readdata(self,filename_):
    """
    Reads problem data from a file.
  
    readdata(self,filename_)
      filename: str|unicode. Input data file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_readdataautoformat(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def readparamfile(self):
    """
    Reads a parameter file.
  
    readparamfile(self)
    """
    res = __library__.MSK_XX_readparamfile(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_str)
  def readsolution(self,whichsol_,filename_):
    """
    Reads a solution from a file.
  
    readsolution(self,whichsol_,filename_)
      whichsol: mosek.soltype. Selects a solution.
      filename: str|unicode. A valid file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_readsolution(self.__nativep,whichsol_,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype))
  def readsummary(self,whichstream_):
    """
    Prints information about last file read.
  
    readsummary(self,whichstream_)
      whichstream: mosek.streamtype. Index of the stream.
    """
    res = __library__.MSK_XX_readsummary(self.__nativep,whichstream_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_int,_make_int,_make_int,_make_long,_make_long)
  def resizetask(self,maxnumcon_,maxnumvar_,maxnumcone_,maxnumanz_,maxnumqnz_):
    """
    Resizes an optimization task.
  
    resizetask(self,maxnumcon_,maxnumvar_,maxnumcone_,maxnumanz_,maxnumqnz_)
      maxnumcon: int. New maximum number of constraints.
      maxnumvar: int. New maximum number of variables.
      maxnumcone: int. New maximum number of cones.
      maxnumanz: long. New maximum number of linear non-zero elements.
      maxnumqnz: long. New maximum number of quadratic non-zeros elements.
    """
    res = __library__.MSK_XX_resizetask(self.__nativep,maxnumcon_,maxnumvar_,maxnumcone_,maxnumanz_,maxnumqnz_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str,_make_int)
  def checkmem(self,file_,line_):
    """
    Checks the memory allocated by the task.
  
    checkmem(self,file_,line_)
      file: str|unicode. File from which the function is called.
      line: int. Line in the file from which the function is called.
    """
    if isinstance(file_,unicode):
      file_ = file_.encode("utf-8")
    res = __library__.MSK_XX_checkmemtask(self.__nativep,file_,line_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def getmemusage(self):
    """
    Obtains information about the amount of memory used by a task.
  
    getmemusage(self)
    returns: meminuse,maxmemuse
      meminuse: long. Amount of memory currently used by the task.
      maxmemuse: long. Maximum amount of memory used by the task until now.
    """
    meminuse_ = ctypes.c_int64()
    meminuse_ = ctypes.c_int64()
    maxmemuse_ = ctypes.c_int64()
    maxmemuse_ = ctypes.c_int64()
    res = __library__.MSK_XX_getmemusagetask(self.__nativep,ctypes.byref(meminuse_),ctypes.byref(maxmemuse_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _meminuse_return_value = meminuse_.value
    _maxmemuse_return_value = maxmemuse_.value
    return (_meminuse_return_value,_maxmemuse_return_value)
  @accepts(_accept_any)
  def setdefaults(self):
    """
    Resets all parameters values.
  
    setdefaults(self)
    """
    res = __library__.MSK_XX_setdefaults(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def solutiondef(self,whichsol_):
    """
    Checks whether a solution is defined.
  
    solutiondef(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    returns: isdef
      isdef: int. Is non-zero if the requested solution is defined.
    """
    isdef_ = ctypes.c_int32()
    isdef_ = ctypes.c_int32()
    res = __library__.MSK_XX_solutiondef(self.__nativep,whichsol_,ctypes.byref(isdef_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _isdef_return_value = isdef_.value
    return (_isdef_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def deletesolution(self,whichsol_):
    """
    Undefines a solution and frees the memory it uses.
  
    deletesolution(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    """
    res = __library__.MSK_XX_deletesolution(self.__nativep,whichsol_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def startstat(self):
    """
    Starts the statistics file.
  
    startstat(self)
    """
    res = __library__.MSK_XX_startstat(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def stopstat(self):
    """
    Stops the statistics file.
  
    stopstat(self)
    """
    res = __library__.MSK_XX_stopstat(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any)
  def appendstat(self):
    """
    Appends a record the statistics file.
  
    appendstat(self)
    """
    res = __library__.MSK_XX_appendstat(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype),_accept_anyenum(soltype))
  def onesolutionsummary(self,whichstream_,whichsol_):
    """
    Prints a short summary for the specified solution.
  
    onesolutionsummary(self,whichstream_,whichsol_)
      whichstream: mosek.streamtype. Index of the stream.
      whichsol: mosek.soltype. Selects a solution.
    """
    res = __library__.MSK_XX_onesolutionsummary(self.__nativep,whichstream_,whichsol_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype))
  def solutionsummary(self,whichstream_):
    """
    Prints a short summary of the current solutions.
  
    solutionsummary(self,whichstream_)
      whichstream: mosek.streamtype. Index of the stream.
    """
    res = __library__.MSK_XX_solutionsummary(self.__nativep,whichstream_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def updatesolutioninfo(self,whichsol_):
    """
    Update the information items related to the solution.
  
    updatesolutioninfo(self,whichsol_)
      whichsol: mosek.soltype. Selects a solution.
    """
    res = __library__.MSK_XX_updatesolutioninfo(self.__nativep,whichsol_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(streamtype))
  def optimizersummary(self,whichstream_):
    """
    Prints a short summary with optimizer statistics for last optimization.
  
    optimizersummary(self,whichstream_)
      whichstream: mosek.streamtype. Index of the stream.
    """
    res = __library__.MSK_XX_optimizersummary(self.__nativep,whichstream_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def strtoconetype(self,str_):
    """
    Obtains a cone type code.
  
    strtoconetype(self,str_)
      str: str|unicode. String corresponding to the cone type code.
    returns: conetype
      conetype: mosek.conetype. The cone type corresponding to str.
    """
    if isinstance(str_,unicode):
      str_ = str_.encode("utf-8")
    conetype_ = ctypes.c_int32()
    res = __library__.MSK_XX_strtoconetype(self.__nativep,str_,ctypes.byref(conetype_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _conetype_return_value = conetype(conetype_.value)
    return (_conetype_return_value)
  @accepts(_accept_any,_accept_str)
  def strtosk(self,str_):
    """
    Obtains a status key.
  
    strtosk(self,str_)
      str: str|unicode. Status key string.
    returns: sk
      sk: int. Status key corresponding to the string.
    """
    if isinstance(str_,unicode):
      str_ = str_.encode("utf-8")
    sk_ = ctypes.c_int32()
    sk_ = ctypes.c_int32()
    res = __library__.MSK_XX_strtosk(self.__nativep,str_,ctypes.byref(sk_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _sk_return_value = sk_.value
    return (_sk_return_value)
  @accepts(_accept_any,_accept_str)
  def writedata(self,filename_):
    """
    Writes problem data to a file.
  
    writedata(self,filename_)
      filename: str|unicode. Output file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_writedata(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def writetask(self,filename_):
    """
    Write a complete binary dump of the task data.
  
    writetask(self,filename_)
      filename: str|unicode. Output file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_writetask(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def readtask(self,filename_):
    """
    Load task data from a file.
  
    readtask(self,filename_)
      filename: str|unicode. Input file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_readtask(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def readbranchpriorities(self,filename_):
    """
    Reads branching priority data from a file.
  
    readbranchpriorities(self,filename_)
      filename: str|unicode. Input file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_readbranchpriorities(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def writebranchpriorities(self,filename_):
    """
    Writes branching priority data to a file.
  
    writebranchpriorities(self,filename_)
      filename: str|unicode. Output file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_writebranchpriorities(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_str)
  def writeparamfile(self,filename_):
    """
    Writes all the parameters to a parameter file.
  
    writeparamfile(self,filename_)
      filename: str|unicode. The name of parameter file.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_writeparamfile(self.__nativep,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_accept_anyenum(soltype))
  def getinfeasiblesubproblem(self,whichsol_):
    """
    Obtains an infeasible sub problem.
  
    getinfeasiblesubproblem(self,whichsol_)
      whichsol: mosek.soltype. Which solution to use when determining the infeasible subproblem.
    returns: 
    """
    inftask_ = ctypes.c_void_p()
    inftask_ = ctypes.c_void_p()
    res = __library__.MSK_XX_getinfeasiblesubproblem(self.__nativep,whichsol_,ctypes.byref(inftask_))
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    _inftask_return_value = Task(nativep = inftask_.value)
    return (_inftask_return_value)
  @accepts(_accept_any,_accept_anyenum(soltype),_accept_str)
  def writesolution(self,whichsol_,filename_):
    """
    Write a solution to a file.
  
    writesolution(self,whichsol_,filename_)
      whichsol: mosek.soltype. Selects a solution.
      filename: str|unicode. A valid file name.
    """
    if isinstance(filename_,unicode):
      filename_ = filename_.encode("utf-8")
    res = __library__.MSK_XX_writesolution(self.__nativep,whichsol_,filename_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_make_anyenumvector(mark),_make_intvector,_make_anyenumvector(mark),_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector)
  def primalsensitivity(self,subi_,marki_,subj_,markj_,leftpricei_,rightpricei_,leftrangei_,rightrangei_,leftpricej_,rightpricej_,leftrangej_,rightrangej_):
    """
    Perform sensitivity analysis on bounds.
  
    primalsensitivity(self,subi_,marki_,subj_,markj_,leftpricei_,rightpricei_,leftrangei_,rightrangei_,leftpricej_,rightpricej_,leftrangej_,rightrangej_)
      subi: array of int. Indexes of bounds on constraints to analyze.
      marki: array of mosek.mark. Mark which constraint bounds to analyze.
      subj: array of int. Indexes of bounds on variables to analyze.
      markj: array of mosek.mark. Mark which variable bounds to analyze.
      leftpricei: array of double. Left shadow price for constraints.
      rightpricei: array of double. Right shadow price for constraints.
      leftrangei: array of double. Left range for constraints.
      rightrangei: array of double. Right range for constraints.
      leftpricej: array of double. Left price for variables.
      rightpricej: array of double. Right price for variables.
      leftrangej: array of double. Left range for variables.
      rightrangej: array of double. Right range for variables.
    """
    numi_ = None
    if numi_ is None:
      numi_ = len(subi_)
    else:
      numi_ = min(numi_,len(subi_))
    if numi_ is None:
      numi_ = len(marki_)
    else:
      numi_ = min(numi_,len(marki_))
    if subi_ is None:
      raise ValueError("Argument subi cannot be None")
    if subi_ is None:
      raise ValueError("Argument subi may not be None")
    if isinstance(subi_, numpy.ndarray) and subi_.dtype is numpy.dtype(numpy.int32) and subi_.flags.contiguous:
      _subi_copyarray = False
      _subi_tmp = ctypes.cast(subi_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subi_,array.ndarray) and subi_.dtype is numpy.dtype(array.int32) and  subi_.getsteplength() == 1:
      _subi_copyarray = False
      _subi_tmp = subi_.getdatacptr()
    elif subi_ is None:
      _subi_copyarray = False
      _subi_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subi_copyarray = True
      _subi_tmp = (ctypes.c_int32 * len(subi_))(*subi_)
    if marki_ is None:
      raise ValueError("Argument marki cannot be None")
    if marki_ is None:
      raise ValueError("Argument marki may not be None")
    if marki_ is not None:
        _marki_tmp = (ctypes.c_int32 * len(marki_))(*marki_)
    else:
        _marki_tmp = None
    numj_ = None
    if numj_ is None:
      numj_ = len(subj_)
    else:
      numj_ = min(numj_,len(subj_))
    if numj_ is None:
      numj_ = len(markj_)
    else:
      numj_ = min(numj_,len(markj_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if markj_ is None:
      raise ValueError("Argument markj cannot be None")
    if markj_ is None:
      raise ValueError("Argument markj may not be None")
    if markj_ is not None:
        _markj_tmp = (ctypes.c_int32 * len(markj_))(*markj_)
    else:
        _markj_tmp = None
    if leftpricei_ is not None and len(leftpricei_) < (numi_):
      raise ValueError("Array argument leftpricei is not long enough")
    if isinstance(leftpricei_,numpy.ndarray) and not leftpricei_.flags.writeable:
      raise ValueError("Argument leftpricei must be writable")
    if isinstance(leftpricei_, numpy.ndarray) and leftpricei_.dtype is numpy.dtype(numpy.float64) and leftpricei_.flags.contiguous:
      _leftpricei_copyarray = False
      _leftpricei_tmp = ctypes.cast(leftpricei_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftpricei_,array.ndarray) and leftpricei_.dtype is numpy.dtype(array.float64) and  leftpricei_.getsteplength() == 1:
      _leftpricei_copyarray = False
      _leftpricei_tmp = leftpricei_.getdatacptr()
    elif leftpricei_ is None:
      _leftpricei_copyarray = False
      _leftpricei_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftpricei_copyarray = True
      _leftpricei_tmp = (ctypes.c_double * len(leftpricei_))(*leftpricei_)
    if rightpricei_ is not None and len(rightpricei_) < (numi_):
      raise ValueError("Array argument rightpricei is not long enough")
    if isinstance(rightpricei_,numpy.ndarray) and not rightpricei_.flags.writeable:
      raise ValueError("Argument rightpricei must be writable")
    if isinstance(rightpricei_, numpy.ndarray) and rightpricei_.dtype is numpy.dtype(numpy.float64) and rightpricei_.flags.contiguous:
      _rightpricei_copyarray = False
      _rightpricei_tmp = ctypes.cast(rightpricei_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightpricei_,array.ndarray) and rightpricei_.dtype is numpy.dtype(array.float64) and  rightpricei_.getsteplength() == 1:
      _rightpricei_copyarray = False
      _rightpricei_tmp = rightpricei_.getdatacptr()
    elif rightpricei_ is None:
      _rightpricei_copyarray = False
      _rightpricei_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightpricei_copyarray = True
      _rightpricei_tmp = (ctypes.c_double * len(rightpricei_))(*rightpricei_)
    if leftrangei_ is not None and len(leftrangei_) < (numi_):
      raise ValueError("Array argument leftrangei is not long enough")
    if isinstance(leftrangei_,numpy.ndarray) and not leftrangei_.flags.writeable:
      raise ValueError("Argument leftrangei must be writable")
    if isinstance(leftrangei_, numpy.ndarray) and leftrangei_.dtype is numpy.dtype(numpy.float64) and leftrangei_.flags.contiguous:
      _leftrangei_copyarray = False
      _leftrangei_tmp = ctypes.cast(leftrangei_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftrangei_,array.ndarray) and leftrangei_.dtype is numpy.dtype(array.float64) and  leftrangei_.getsteplength() == 1:
      _leftrangei_copyarray = False
      _leftrangei_tmp = leftrangei_.getdatacptr()
    elif leftrangei_ is None:
      _leftrangei_copyarray = False
      _leftrangei_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftrangei_copyarray = True
      _leftrangei_tmp = (ctypes.c_double * len(leftrangei_))(*leftrangei_)
    if rightrangei_ is not None and len(rightrangei_) < (numi_):
      raise ValueError("Array argument rightrangei is not long enough")
    if isinstance(rightrangei_,numpy.ndarray) and not rightrangei_.flags.writeable:
      raise ValueError("Argument rightrangei must be writable")
    if isinstance(rightrangei_, numpy.ndarray) and rightrangei_.dtype is numpy.dtype(numpy.float64) and rightrangei_.flags.contiguous:
      _rightrangei_copyarray = False
      _rightrangei_tmp = ctypes.cast(rightrangei_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightrangei_,array.ndarray) and rightrangei_.dtype is numpy.dtype(array.float64) and  rightrangei_.getsteplength() == 1:
      _rightrangei_copyarray = False
      _rightrangei_tmp = rightrangei_.getdatacptr()
    elif rightrangei_ is None:
      _rightrangei_copyarray = False
      _rightrangei_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightrangei_copyarray = True
      _rightrangei_tmp = (ctypes.c_double * len(rightrangei_))(*rightrangei_)
    if leftpricej_ is not None and len(leftpricej_) < (numj_):
      raise ValueError("Array argument leftpricej is not long enough")
    if isinstance(leftpricej_,numpy.ndarray) and not leftpricej_.flags.writeable:
      raise ValueError("Argument leftpricej must be writable")
    if isinstance(leftpricej_, numpy.ndarray) and leftpricej_.dtype is numpy.dtype(numpy.float64) and leftpricej_.flags.contiguous:
      _leftpricej_copyarray = False
      _leftpricej_tmp = ctypes.cast(leftpricej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftpricej_,array.ndarray) and leftpricej_.dtype is numpy.dtype(array.float64) and  leftpricej_.getsteplength() == 1:
      _leftpricej_copyarray = False
      _leftpricej_tmp = leftpricej_.getdatacptr()
    elif leftpricej_ is None:
      _leftpricej_copyarray = False
      _leftpricej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftpricej_copyarray = True
      _leftpricej_tmp = (ctypes.c_double * len(leftpricej_))(*leftpricej_)
    if rightpricej_ is not None and len(rightpricej_) < (numj_):
      raise ValueError("Array argument rightpricej is not long enough")
    if isinstance(rightpricej_,numpy.ndarray) and not rightpricej_.flags.writeable:
      raise ValueError("Argument rightpricej must be writable")
    if isinstance(rightpricej_, numpy.ndarray) and rightpricej_.dtype is numpy.dtype(numpy.float64) and rightpricej_.flags.contiguous:
      _rightpricej_copyarray = False
      _rightpricej_tmp = ctypes.cast(rightpricej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightpricej_,array.ndarray) and rightpricej_.dtype is numpy.dtype(array.float64) and  rightpricej_.getsteplength() == 1:
      _rightpricej_copyarray = False
      _rightpricej_tmp = rightpricej_.getdatacptr()
    elif rightpricej_ is None:
      _rightpricej_copyarray = False
      _rightpricej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightpricej_copyarray = True
      _rightpricej_tmp = (ctypes.c_double * len(rightpricej_))(*rightpricej_)
    if leftrangej_ is not None and len(leftrangej_) < (numj_):
      raise ValueError("Array argument leftrangej is not long enough")
    if isinstance(leftrangej_,numpy.ndarray) and not leftrangej_.flags.writeable:
      raise ValueError("Argument leftrangej must be writable")
    if isinstance(leftrangej_, numpy.ndarray) and leftrangej_.dtype is numpy.dtype(numpy.float64) and leftrangej_.flags.contiguous:
      _leftrangej_copyarray = False
      _leftrangej_tmp = ctypes.cast(leftrangej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftrangej_,array.ndarray) and leftrangej_.dtype is numpy.dtype(array.float64) and  leftrangej_.getsteplength() == 1:
      _leftrangej_copyarray = False
      _leftrangej_tmp = leftrangej_.getdatacptr()
    elif leftrangej_ is None:
      _leftrangej_copyarray = False
      _leftrangej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftrangej_copyarray = True
      _leftrangej_tmp = (ctypes.c_double * len(leftrangej_))(*leftrangej_)
    if rightrangej_ is not None and len(rightrangej_) < (numj_):
      raise ValueError("Array argument rightrangej is not long enough")
    if isinstance(rightrangej_,numpy.ndarray) and not rightrangej_.flags.writeable:
      raise ValueError("Argument rightrangej must be writable")
    if isinstance(rightrangej_, numpy.ndarray) and rightrangej_.dtype is numpy.dtype(numpy.float64) and rightrangej_.flags.contiguous:
      _rightrangej_copyarray = False
      _rightrangej_tmp = ctypes.cast(rightrangej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightrangej_,array.ndarray) and rightrangej_.dtype is numpy.dtype(array.float64) and  rightrangej_.getsteplength() == 1:
      _rightrangej_copyarray = False
      _rightrangej_tmp = rightrangej_.getdatacptr()
    elif rightrangej_ is None:
      _rightrangej_copyarray = False
      _rightrangej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightrangej_copyarray = True
      _rightrangej_tmp = (ctypes.c_double * len(rightrangej_))(*rightrangej_)
    res = __library__.MSK_XX_primalsensitivity(self.__nativep,numi_,_subi_tmp,_marki_tmp,numj_,_subj_tmp,_markj_tmp,_leftpricei_tmp,_rightpricei_tmp,_leftrangei_tmp,_rightrangei_tmp,_leftpricej_tmp,_rightpricej_tmp,_leftrangej_tmp,_rightrangej_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _leftpricei_copyarray:
      if isinstance(leftpricei_, numpy.ndarray) and False:
        leftpricei_[:] = list(_leftpricei_tmp)
      else:
        leftpricei_[:] = _leftpricei_tmp
    if _rightpricei_copyarray:
      if isinstance(rightpricei_, numpy.ndarray) and False:
        rightpricei_[:] = list(_rightpricei_tmp)
      else:
        rightpricei_[:] = _rightpricei_tmp
    if _leftrangei_copyarray:
      if isinstance(leftrangei_, numpy.ndarray) and False:
        leftrangei_[:] = list(_leftrangei_tmp)
      else:
        leftrangei_[:] = _leftrangei_tmp
    if _rightrangei_copyarray:
      if isinstance(rightrangei_, numpy.ndarray) and False:
        rightrangei_[:] = list(_rightrangei_tmp)
      else:
        rightrangei_[:] = _rightrangei_tmp
    if _leftpricej_copyarray:
      if isinstance(leftpricej_, numpy.ndarray) and False:
        leftpricej_[:] = list(_leftpricej_tmp)
      else:
        leftpricej_[:] = _leftpricej_tmp
    if _rightpricej_copyarray:
      if isinstance(rightpricej_, numpy.ndarray) and False:
        rightpricej_[:] = list(_rightpricej_tmp)
      else:
        rightpricej_[:] = _rightpricej_tmp
    if _leftrangej_copyarray:
      if isinstance(leftrangej_, numpy.ndarray) and False:
        leftrangej_[:] = list(_leftrangej_tmp)
      else:
        leftrangej_[:] = _leftrangej_tmp
    if _rightrangej_copyarray:
      if isinstance(rightrangej_, numpy.ndarray) and False:
        rightrangej_[:] = list(_rightrangej_tmp)
      else:
        rightrangej_[:] = _rightrangej_tmp
  @accepts(_accept_any,_accept_anyenum(streamtype))
  def sensitivityreport(self,whichstream_):
    """
    Creates a sensitivity report.
  
    sensitivityreport(self,whichstream_)
      whichstream: mosek.streamtype. Index of the stream.
    """
    res = __library__.MSK_XX_sensitivityreport(self.__nativep,whichstream_)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
  @accepts(_accept_any,_make_intvector,_accept_doublevector,_accept_doublevector,_accept_doublevector,_accept_doublevector)
  def dualsensitivity(self,subj_,leftpricej_,rightpricej_,leftrangej_,rightrangej_):
    """
    Performs sensitivity analysis on objective coefficients.
  
    dualsensitivity(self,subj_,leftpricej_,rightpricej_,leftrangej_,rightrangej_)
      subj: array of int. Index of objective coefficients to analyze.
      leftpricej: array of double. Left shadow prices for requested coefficients.
      rightpricej: array of double. Right shadow prices for requested coefficients.
      leftrangej: array of double. Left range for requested coefficients.
      rightrangej: array of double. Right range for requested coefficients.
    """
    numj_ = None
    if numj_ is None:
      numj_ = len(subj_)
    else:
      numj_ = min(numj_,len(subj_))
    if subj_ is None:
      raise ValueError("Argument subj cannot be None")
    if subj_ is None:
      raise ValueError("Argument subj may not be None")
    if isinstance(subj_, numpy.ndarray) and subj_.dtype is numpy.dtype(numpy.int32) and subj_.flags.contiguous:
      _subj_copyarray = False
      _subj_tmp = ctypes.cast(subj_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_int32))
    elif isinstance(subj_,array.ndarray) and subj_.dtype is numpy.dtype(array.int32) and  subj_.getsteplength() == 1:
      _subj_copyarray = False
      _subj_tmp = subj_.getdatacptr()
    elif subj_ is None:
      _subj_copyarray = False
      _subj_tmp = ctypes.POINTER(ctypes.c_int32)()
    else:
      _subj_copyarray = True
      _subj_tmp = (ctypes.c_int32 * len(subj_))(*subj_)
    if leftpricej_ is not None and len(leftpricej_) < (numj_):
      raise ValueError("Array argument leftpricej is not long enough")
    if isinstance(leftpricej_,numpy.ndarray) and not leftpricej_.flags.writeable:
      raise ValueError("Argument leftpricej must be writable")
    if isinstance(leftpricej_, numpy.ndarray) and leftpricej_.dtype is numpy.dtype(numpy.float64) and leftpricej_.flags.contiguous:
      _leftpricej_copyarray = False
      _leftpricej_tmp = ctypes.cast(leftpricej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftpricej_,array.ndarray) and leftpricej_.dtype is numpy.dtype(array.float64) and  leftpricej_.getsteplength() == 1:
      _leftpricej_copyarray = False
      _leftpricej_tmp = leftpricej_.getdatacptr()
    elif leftpricej_ is None:
      _leftpricej_copyarray = False
      _leftpricej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftpricej_copyarray = True
      _leftpricej_tmp = (ctypes.c_double * len(leftpricej_))(*leftpricej_)
    if rightpricej_ is not None and len(rightpricej_) < (numj_):
      raise ValueError("Array argument rightpricej is not long enough")
    if isinstance(rightpricej_,numpy.ndarray) and not rightpricej_.flags.writeable:
      raise ValueError("Argument rightpricej must be writable")
    if isinstance(rightpricej_, numpy.ndarray) and rightpricej_.dtype is numpy.dtype(numpy.float64) and rightpricej_.flags.contiguous:
      _rightpricej_copyarray = False
      _rightpricej_tmp = ctypes.cast(rightpricej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightpricej_,array.ndarray) and rightpricej_.dtype is numpy.dtype(array.float64) and  rightpricej_.getsteplength() == 1:
      _rightpricej_copyarray = False
      _rightpricej_tmp = rightpricej_.getdatacptr()
    elif rightpricej_ is None:
      _rightpricej_copyarray = False
      _rightpricej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightpricej_copyarray = True
      _rightpricej_tmp = (ctypes.c_double * len(rightpricej_))(*rightpricej_)
    if leftrangej_ is not None and len(leftrangej_) < (numj_):
      raise ValueError("Array argument leftrangej is not long enough")
    if isinstance(leftrangej_,numpy.ndarray) and not leftrangej_.flags.writeable:
      raise ValueError("Argument leftrangej must be writable")
    if isinstance(leftrangej_, numpy.ndarray) and leftrangej_.dtype is numpy.dtype(numpy.float64) and leftrangej_.flags.contiguous:
      _leftrangej_copyarray = False
      _leftrangej_tmp = ctypes.cast(leftrangej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(leftrangej_,array.ndarray) and leftrangej_.dtype is numpy.dtype(array.float64) and  leftrangej_.getsteplength() == 1:
      _leftrangej_copyarray = False
      _leftrangej_tmp = leftrangej_.getdatacptr()
    elif leftrangej_ is None:
      _leftrangej_copyarray = False
      _leftrangej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _leftrangej_copyarray = True
      _leftrangej_tmp = (ctypes.c_double * len(leftrangej_))(*leftrangej_)
    if rightrangej_ is not None and len(rightrangej_) < (numj_):
      raise ValueError("Array argument rightrangej is not long enough")
    if isinstance(rightrangej_,numpy.ndarray) and not rightrangej_.flags.writeable:
      raise ValueError("Argument rightrangej must be writable")
    if isinstance(rightrangej_, numpy.ndarray) and rightrangej_.dtype is numpy.dtype(numpy.float64) and rightrangej_.flags.contiguous:
      _rightrangej_copyarray = False
      _rightrangej_tmp = ctypes.cast(rightrangej_.ctypes._as_parameter_,ctypes.POINTER(ctypes.c_double))
    elif isinstance(rightrangej_,array.ndarray) and rightrangej_.dtype is numpy.dtype(array.float64) and  rightrangej_.getsteplength() == 1:
      _rightrangej_copyarray = False
      _rightrangej_tmp = rightrangej_.getdatacptr()
    elif rightrangej_ is None:
      _rightrangej_copyarray = False
      _rightrangej_tmp = ctypes.POINTER(ctypes.c_double)()
    else:
      _rightrangej_copyarray = True
      _rightrangej_tmp = (ctypes.c_double * len(rightrangej_))(*rightrangej_)
    res = __library__.MSK_XX_dualsensitivity(self.__nativep,numj_,_subj_tmp,_leftpricej_tmp,_rightpricej_tmp,_leftrangej_tmp,_rightrangej_tmp)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)
    if _leftpricej_copyarray:
      if isinstance(leftpricej_, numpy.ndarray) and False:
        leftpricej_[:] = list(_leftpricej_tmp)
      else:
        leftpricej_[:] = _leftpricej_tmp
    if _rightpricej_copyarray:
      if isinstance(rightpricej_, numpy.ndarray) and False:
        rightpricej_[:] = list(_rightpricej_tmp)
      else:
        rightpricej_[:] = _rightpricej_tmp
    if _leftrangej_copyarray:
      if isinstance(leftrangej_, numpy.ndarray) and False:
        leftrangej_[:] = list(_leftrangej_tmp)
      else:
        leftrangej_[:] = _leftrangej_tmp
    if _rightrangej_copyarray:
      if isinstance(rightrangej_, numpy.ndarray) and False:
        rightrangej_[:] = list(_rightrangej_tmp)
      else:
        rightrangej_[:] = _rightrangej_tmp
  @accepts(_accept_any)
  def checkconvexity(self):
    """
    Checks if a quadratic optimization problem is convex.
  
    checkconvexity(self)
    """
    res = __library__.MSK_XX_checkconvexity(self.__nativep)
    if res != 0:
      _,msg = self.__getlasterror(res)
      raise Error(rescode(res),msg)


if __name__ == '__main__':
    env = Env()
