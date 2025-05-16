'''
Copyright © 2016 Till Ehrengruber
Copyright © 2025 Seneral

Distributed under the GNU GENERAL PUBLIC LICENSE VERSION 3.0.

Source: https://github.com/tehrengruber/LLDB-Eigen-Data-Formatter

Modified by Seneral:
1. Provide inline formatting for small matrices
2. Reduce code branching
Attempted modificaitons:
3. Added eigen_matrix_element_provider to provide element enumeration (disabled, doesn't work)
4. Added support for both ColMajor and RowMajor matrices (disabled, doesn't work)
'''

import lldb
import os
import sys

# Define a context manager to suppress stdout and stderr.
#  see http://stackoverflow.com/questions/11130156/suppress-stdout-stderr-print-from-python-functions
class suppress_stdout_stderr(object):
    def __init__(self):
        # Open a pair of null files
        self.null_fds =  [os.open(os.devnull,os.O_RDWR) for x in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors.
        self.save_fds = (os.dup(1), os.dup(2))

    def __enter__(self):
        # Assign the null pointers to stdout and stderr.
        os.dup2(self.null_fds[0],1)
        os.dup2(self.null_fds[1],2)

    def __exit__(self, *_):
        # Re-assign the real stdout/stderr back to (1) and (2)
        os.dup2(self.save_fds[0],1)
        os.dup2(self.save_fds[1],2)
        # Close the null files
        os.close(self.null_fds[0])
        os.close(self.null_fds[1])

def evaluate_expression(valobj, expr):
    return valobj.GetProcess().GetSelectedThread().GetSelectedFrame().EvaluateExpression(expr)

def _row_element(valobj, row, rows, cols, rowStride, colStride):
    for i in range(0, cols):
        yield valobj.GetChildAtIndex(row*rowStride + i*colStride, lldb.eNoDynamicValues, True).GetValue()

def print_raw_matrix(valobj, rows, cols, rowStride, colStride):

    # check that the data layout fits a regular dense matrix
    if rows*cols != valobj.GetNumChildren():
        logger = lldb.formatters.Logger.Logger()
        logger >> "Rows and Cols (%d, %d) mismatch with size %d " % (rows, cols, valobj.GetNumChildren())
        return "<" + str(valobj.GetNumChildren()) + ">" + "[ unknown layout ]"

    # print matrix dimensions
    output = "<" + str(rows) + "," + str(cols) + "> "

    # try to access last value (if this throws an exception the matrix is probably not declared yet)
    try:
        valobj.GetChildAtIndex(rows*cols, lldb.eNoDynamicValues, True).GetValue()
    except:
        return output + "[ inaccessible ]"

    if rows*cols > 100:
        return output + "[ too large ]"

    if rows*cols <= 8:
        # Inline small matrices and vectors
        output += "[ "
        for j in range(0, rows):
            output += " ".join(_row_element(valobj, j, rows, cols, rowStride, colStride)) + (", " if j < rows-1 else "")
        return output + " ]\n"

    output += "[\n"

    # determine padding
    padding = 1
    for i in range(0, rows*cols):
        padding = max(padding, len(str(valobj.GetChildAtIndex(i, lldb.eNoDynamicValues, True).GetValue())))

    # print values
    for j in range(0, rows):
        output += "".join(val.rjust(padding+1, ' ') for val in _row_element(valobj, j, rows, cols, rowStride, colStride)) + "\n"
        
    return output + "] "

def get_matrix_data(valobj):
    # determine type
    if valobj.GetValueForExpressionPath(".m_storage.m_data.array").IsValid():
        return valobj.GetValueForExpressionPath(".m_storage.m_data.array")
    elif valobj.GetValueForExpressionPath(".m_storage.m_data").GetType().IsPointerType():
        return valobj.GetValueForExpressionPath(".m_storage.m_data")
    else:
        return None

def format_eigen_matrix(valobj,internal_dict):
    data = get_matrix_data(valobj)

    if not data or not data.IsValid():
        return valobj.GetSummary()

    # determine expression path of the current valobj
    stream = lldb.SBStream()
    valobj.GetExpressionPath(stream)
    valobj_expression_path = stream.GetData()

    logger = lldb.formatters.Logger.Logger()

    # determine rows and cols
    rows = cols = 0
    rowStride = colStride = 0
    with suppress_stdout_stderr():
        # todo: check result is valid
        rows = evaluate_expression(valobj, valobj_expression_path+".rows()").GetValueAsSigned()
        cols = evaluate_expression(valobj, valobj_expression_path+".cols()").GetValueAsSigned()
        #rowStride = evaluate_expression(valobj, valobj_expression_path+".rowStride()").GetValueAsUnsigned()
        #colStride = evaluate_expression(valobj, valobj_expression_path+".colStride()").GetValueAsUnsigned()
        rowStride = 1
        colStride = rows
        logger >> "'%s' has size %dx%d and stride %dx%d!" % (valobj_expression_path, rows, cols, rowStride, colStride)
        #print(  , file=sys.stderr)
        #sys.stderr.flush()

    return print_raw_matrix(data, rows, cols, rowStride, colStride)


class eigen_matrix_element_provider:
    def __init__(self, valobj, dict):
        self.valobj = valobj
        self.data = None
        self.rows = None
        self.cols = None
        self.rowStride = None
        self.colStride = None

    def update(self):
        logger = lldb.formatters.Logger.Logger()
        try:
            self.data = get_matrix_data(self.valobj)
            self.rows = evaluate_expression(self.data, valobj_expression_path+".rows()").GetValueAsSigned()
            self.cols = evaluate_expression(self.data, valobj_expression_path+".cols()").GetValueAsSigned()
            self.rowStride = evaluate_expression(self.data, valobj_expression_path+".rowStride()").GetValueAsSigned()
            self.colStride = evaluate_expression(self.data, valobj_expression_path+".colStride()").GetValueAsSigned()
            logger >> "Updated: row %d col %d" % (row, col)
        except Exception as e:
            logger >> "Caught exception: %r" % e
            pass

    def get_type_name(self):
        if not self.data:
            self.update()
        if not self.data:
            return self.valobj.GetSummary()
        return print_raw_matrix(self.data, self.rows, self.cols, self.rowStride, self.colStride)

    def num_children(self):
        if not self.data:
            self.update()
        if not self.data:
            return 0
        return self.data.GetNumChildren()

    def has_children(self):
        if not self.data:
            self.update()
        if not self.data:
            return False
        return self.rows*self.cols > 0

    def get_child_index(self, name):
        logger = lldb.formatters.Logger.Logger()
        try:
            rem = name.lstrip("[").rstrip("]").split(",")
            if (len(rem) > 1):
                return int(rem[0]) * self.rowStride + int(rem[1]) * self.colStride
            return int(rem[0])
        except:
            return -1

    def get_child_at_index(self, index):
        logger = lldb.formatters.Logger.Logger()
        logger >> "Retrieving child " + str(index)
        if not self.data:
            self.update()
        if index < 0:
            return None
        if index >= self.num_children():
            return None

        # hit the index! so we have the value
        value = self.data.GetChildAtIndex(index, lldb.eNoDynamicValues, True).GetValue()
        if self.cols == 1 or self.rows == 1:
            return self.valobj.CreateValueFromData(
                "[%d] %f" % (index, value), value.GetData(), value.GetType()
            )
        else:
            row = index / self.rowStride if self.rowStride > 1 else index % self.colStride
            col = index / self.colStride if self.colStride > 1 else index % self.rowStride
            logger >> " : row %d col %d" % (row, col)
            return self.valobj.CreateValueFromData(
                "[%d,%d] %f" % (row, col, value), value.GetData(), value.GetType()
            )

def __lldb_init_module (debugger, dict):
    debugger.HandleCommand("type summary add -x \"Eigen::Matrix\" -F lldb_formatters.format_eigen_matrix")
    debugger.HandleCommand("type summary add -x \"Eigen::Array\" -F lldb_formatters.format_eigen_matrix")
    #debugger.HandleCommand("type synthetic add -x \"Eigen::Matrix\" -l lldb_formatters.eigen_matrix_element_provider")
    #debugger.HandleCommand("type synthetic add -x \"Eigen::Array\" -l lldb_formatters.eigen_matrix_element_provider")