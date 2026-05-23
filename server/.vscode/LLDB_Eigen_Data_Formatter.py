'''
Copyright © 2026 Seneral

Distributed under the GNU GENERAL PUBLIC LICENSE VERSION 3.0.

Supported:
- Dense Matrices, Arrays, Refs and Blocks
- Sparse Matrices and Vectors, with basic support for Sparse Blocks
- Supports dynamic and compile-time sizes, storage orders and strides
- Inline dense-formatted summary (truncated for larger matrices)
- Synthetic children to iterate full matrix with proper indexing (except Sparse Blocks)

Not Supported / Tested:
- Tensors
- Ref of Sparse Matrices
- Sparse Block Synthetic Children

Does not use any function evaluations
- May not work when function is optimised out
- May slow down debugging experience considerably
- Instead parses just fields, works as long as data is accessible to LLDB
'''

import lldb
import os
import re

def get_expression_path(valobj):
	# Determine expression path of the valobj
	stream = lldb.SBStream()
	valobj.GetExpressionPath(stream)
	return stream.GetData()

def do_log(valobj, msg):
	# Log with context of object
	lldb.formatters.Logger.Logger() >> "'" + get_expression_path(valobj) + "': " + msg

def debug_templates(valobj, t, tgt):
	for i in range(t.GetNumberOfTemplateArguments()):
		if t.GetTemplateArgumentKind(i) == lldb.eTemplateArgumentKindType:
			do_log(valobj, "Template parameter " + str(i) + ": type " + t.GetTemplateArgumentType(i).GetName())
		else:
			do_log(valobj, "Template parameter " + str(i) + ": value " + t.GetTemplateArgumentValue(tgt, i).GetValue())

def find_base_class(t, base):
	if t.GetName().startswith(base):
		return t
	for b in range(t.GetNumberOfDirectBaseClasses()):
		res = find_base_class(t.GetDirectBaseClassAtIndex(b).GetType(), base)
		if res is not None: return res
	return None

def get_clean_type(t):
	t = t.GetUnqualifiedType().GetCanonicalType()
	if t.IsReferenceType(): t = t.GetDereferencedType()
	if t.IsPointerType(): t = t.GetPointeeType()
	return t

def get_toplevel_eigen_type(valobj):
	if not valobj: return None
	t = get_clean_type(valobj.GetType())
	name = re.match("^Eigen::(.+?)<", t.GetName())
	if not name:
		lldb.formatters.Logger.Logger() >> "Failed to match " + str(t)
	return name.group(1) if name else None

class eigen_dense_matrix_array_ref:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.data = None
		self.rows = self.cols = self.rowStride = self.colStride = 0
		self.size = -1
		self.rowMajor = False
		self.is_pointer = False
		self.elementType = None
		self.typeName = get_toplevel_eigen_type(self.valobj)
		self.is_copied = False # For Ref only
		self.valid = self.typeName in ["Matrix", "Array", "Ref"]

	def update_matrix_array(self, valobj):
		# Get storage object and the actual data
		storage = valobj.GetValueForExpressionPath(".m_storage")
		if not storage.IsValid():
			do_log(valobj, "No storage member!")
			return
		self.data = storage.GetValueForExpressionPath(".m_data.array")
		if not self.data.IsValid():
			self.data = storage.GetValueForExpressionPath(".m_data")
		if not self.data.IsValid():
			do_log(valobj, "No valid data member!")
			return

		# Get typed layout information
		tgt = storage.GetTarget()
		t = get_clean_type(storage.GetType())
		try:
			self.size = int(t.GetTemplateArgumentValue(tgt, 1).GetValue())
			self.rows = int(t.GetTemplateArgumentValue(tgt, 2).GetValue())
			self.cols = int(t.GetTemplateArgumentValue(tgt, 3).GetValue())
			opts = int(t.GetTemplateArgumentValue(tgt, 4).GetValue())
		except:
			do_log(valobj, "Failed to parse Matrix/Array template parameters!")
			debug_templates(valobj, t, tgt)
			self.size = -1
			return

		# Get dynamic layout information
		if self.rows <= 0 and storage.GetValueForExpressionPath(".m_rows").IsValid():
			self.rows = storage.GetValueForExpressionPath(".m_rows").GetValueAsSigned()
		if self.cols <= 0 and storage.GetValueForExpressionPath(".m_cols").IsValid():
			self.cols = storage.GetValueForExpressionPath(".m_cols").GetValueAsSigned()

		# Initialise or parse strides
		rowMajor = opts & 1
		self.rowStride = self.cols if rowMajor else 1
		self.colStride = 1 if rowMajor else self.rows
		self.rowMajor = self.colStride < self.rowStride

	def update_ref(self, valobj):
		# Get the actual data
		self.data = valobj.GetValueForExpressionPath(".m_data")
		if not self.data.IsValid():
			do_log(valobj, "No valid data member!")
			return
		outerStride = innerStride = 1

		# Get typed layout information
		tgt = valobj.GetTarget()
		t = get_clean_type(valobj.GetType())
		try:
			refType = t.GetTemplateArgumentType(0)
			self.rows = int(refType.GetTemplateArgumentValue(tgt, 1).GetValue())
			self.cols = int(refType.GetTemplateArgumentValue(tgt, 2).GetValue())
			opts = int(refType.GetTemplateArgumentValue(tgt, 3).GetValue())

			strideType = t.GetTemplateArgumentType(2)
			# Strangely, strideType template arguments NOR the base class Stride is accessible at all, so need to parse string
			outer = re.match("^Eigen::OuterStride<([0-9]+)>", strideType.GetName())
			inner = re.match("^Eigen::InnterStride<([0-9]+)>", strideType.GetName())
			both = re.match("^Eigen::StrideStride<([0-9]+),\s?([0-9]+)>", strideType.GetName())
			if outer:
				outerStride = int(outer.group(1))
			elif inner:
				innerStride = int(inner.group(1))
			elif both:
				outerStride = int(both.group(1))
				innerStride = int(both.group(2))
			if outerStride == 0: outerStride = 1
			if innerStride == 0: innerStride = 1
		except:
			do_log(valobj, "Failed to parse Ref template parameters! " + str(t.GetName()))
			debug_templates(valobj, t, tgt)
			return

		#self.update_matrix_array(self.valobj)

		# Get dynamic layout information
		if self.rows < 0 and valobj.GetValueForExpressionPath(".m_rows.m_value").IsValid():
			self.rows = valobj.GetValueForExpressionPath(".m_rows.m_value").GetValueAsSigned()
		if self.cols < 0 and valobj.GetValueForExpressionPath(".m_cols.m_value").IsValid():
			self.cols = valobj.GetValueForExpressionPath(".m_cols.m_value").GetValueAsSigned()

		# Initialise or parse strides
		rowMajor = opts & 1
		if valobj.GetValueForExpressionPath(".m_stride.m_outer.m_value").IsValid():
			outerStride = valobj.GetValueForExpressionPath(".m_stride.m_outer.m_value").GetValueAsSigned()
		if valobj.GetValueForExpressionPath(".m_stride.m_inner.m_value").IsValid():
			innerStride = valobj.GetValueForExpressionPath(".m_stride.m_inner.m_value").GetValueAsSigned()
		self.rowStride = outerStride if rowMajor else innerStride
		self.colStride = innerStride if rowMajor else outerStride

		# Ref data may in certain scenarios be copied (e.g. some Ref to const data)
		copyData = valobj.GetValueForExpressionPath(".m_object.m_storage.m_data.array")
		if copyData.IsValid():
			self.is_copied = copyData.GetNumChildren() > 0
		else:
			copyData = valobj.GetValueForExpressionPath(".m_object.m_storage.m_data")
			self.is_copied = copyData.IsValid() and copyData.GetValueAsUnsigned() != 0

	def update(self):
		if not self.valid: return
		self.size = -1

		if self.typeName == "Ref":
			self.update_ref(self.valobj)
		else:
			self.update_matrix_array(self.valobj)

		# Preload remaining properties
		self.is_pointer = self.data.GetType().IsPointerType()
		self.elementType = (self.data.GetType().GetPointeeType() if self.is_pointer else self.data.GetType().GetArrayElementType()).GetCanonicalType()

		do_log(self.valobj, ("Dynamic " if self.size < 0 else "Static ") +
			self.elementType.GetName() + " " + self.typeName + (" of size %d x %d" % (self.rows, self.cols)) +
			("" if self.rowStride == 1 and self.colStride == self.rows else
				(", row-major" if self.rowStride == self.cols and self.colStride == 1 else
					(", stride %d, %d" % (self.rowStride, self.colStride)))))
		if self.is_pointer != (self.size < 0):
			do_log(self.valobj, ("Data is pointer" if self.is_pointer else "Data is not pointer") + " but size is " + str(self.size) + "  -----")

		self.size = self.rows*self.cols

	def get_type_name(self):
		return self.valobj.GetType().GetName()

	def get_dimension_label(self):
		if self.size < 0: self.update()
		if self.size < 0: return "[invalid]"
		typeSign = ""
		if self.typeName == "Ref":
			typeSign = "COPY" if self.is_copied else "*"
		if self.rowStride == 1 and self.colStride == self.rows:
			return typeSign + ("(%d,%d)" if self.is_pointer else "<%d,%d>") % (self.rows, self.cols)
		else: # Likely using RowMajor, perhaps actually using stride
			return typeSign + ("(%d,%d,%d,%d)" if self.is_pointer else "<%d,%d,%d,%d>") % (self.rows, self.cols, self.rowStride, self.colStride)

	def has_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return False
		if self.size == 0: return False
		try: # if this throws an exception the matrix is probably not initialised yet
			index = (self.rows-1)*self.rowStride + (self.cols-1)*self.colStride
			if self.is_pointer:
				self.data.GetPointeeData(index, 1)
			else:
				self.data.GetChildAtIndex(index, lldb.eNoDynamicValues, True)
		except:
			return False
		return True

	def num_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return 0
		return self.size

	def get_child_index(self, name):
		try:
			rem = name.lstrip("[").rstrip("]").split(",")
			do_log(self.valobj, "Getting child index of " + name + " ------- UNTESTED!")
			if (len(rem) <= 1): return int(rem[0])
			if self.colStride > self.rowStride: # RowMajor
				do_log(self.valobj, "Accessing as index " + str(int(rem[1]) * self.rows + int(rem[0])) + " in row major matrix!")
				return int(rem[1]) * self.rows + int(rem[0])
			else: # Default, ColMajor
				do_log(self.valobj, "Accessing as index " + str(int(rem[0]) * self.cols + int(rem[1])) + " in col major matrix!")
				return int(rem[0]) * self.cols + int(rem[1])
		except:
			return -1

	def get_element(self, row, col):
		if self.size < 0: return None

		# Access value in actual data buffer with strides
		index = row*self.rowStride + col*self.colStride
		if self.is_pointer:
			element = self.data.GetPointeeData(index, 1)
		else:
			element = self.data.GetChildAtIndex(index, lldb.eNoDynamicValues, True).GetData()

		# Get value as it was indexed
		if self.cols == 1 or self.rows == 1:
			value = self.valobj.CreateValueFromData("[%d]" % (max(row, col)), element, self.elementType)
		else:
			value = self.valobj.CreateValueFromData("[%d,%d]" % (row, col), element, self.elementType)
		return value

	def get_child_at_index(self, index):
		if self.size < 0: self.update()
		if self.size < 0: return None
		if index < 0: return None
		if index >= self.size: return None

		# Parse as if indexed into continuous buffer [0-size]
		if self.colStride < self.rowStride: # RowMajor
			row = int(index / self.cols)
			col = int(index % self.cols)
		else: # Default, ColMajor
			row = int(index % self.rows)
			col = int(index / self.rows)

		return self.get_element(row, col)

class eigen_dense_block:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.is_sparse = valobj.GetValueForExpressionPath(".m_matrix").IsValid()
		if self.is_sparse:
			self.expression = eigen_sparse_matrix(valobj.GetValueForExpressionPath(".m_matrix"), internal_dict)
		else:
			self.expression = eigen_dense_matrix_array_ref(valobj.GetValueForExpressionPath(".m_xpr"), internal_dict)
		self.startRow = self.startCol = self.rows = self.cols = 0
		self.size, self.nnzs = -1, 0
		self.valid = get_toplevel_eigen_type(self.valobj) in ["Block"]

	def update(self):
		if not self.valid: return
		self.size = -1

		self.expression.update()
		if not self.expression.has_children():
			return

		# Get typed layout information
		tgt = self.valobj.GetTarget()
		t = find_base_class(get_clean_type(self.valobj.GetType()), "Eigen::BlockImpl")
		if t is None:
			do_log(self.valobj, "Not a BlockImpl!")
			return
		try:
			self.rows = int(t.GetTemplateArgumentValue(tgt, 1).GetValue())
			self.cols = int(t.GetTemplateArgumentValue(tgt, 2).GetValue())
		except:
			do_log(self.valobj, "Failed to parse BlockImpl template parameters!")
			debug_templates(self.valobj, t, tgt)
			return

		# Get dynamic layout information
		# For dense blocks, and sparse row/col-blocks on col/row-major sparse matrices
		self.startRow = self.valobj.GetValueForExpressionPath(".m_startRow.m_value").GetValueAsSigned()
		self.startCol = self.valobj.GetValueForExpressionPath(".m_startCol.m_value").GetValueAsSigned()
		# For sparse col/row-blocks on col/row-major sparse matrices
		outerStart = self.valobj.GetValueForExpressionPath(".m_outerStart")
		innerStart = self.valobj.GetValueForExpressionPath(".m_innerStart")
		if self.startRow <= 0 and self.expression.rowMajor and outerStart.IsValid():
			self.startRow = outerStart.GetValueAsSigned()
		if self.startRow <= 0 and not self.expression.rowMajor and innerStart.IsValid():
			self.startRow = innerStart.GetValueAsSigned()
		if self.startCol <= 0 and not self.expression.rowMajor and outerStart.IsValid():
			self.startCol = outerStart.GetValueAsSigned()
		if self.startCol <= 0 and self.expression.rowMajor and innerStart.IsValid():
			self.startCol = innerStart.GetValueAsSigned()

		# For dense blocks, and sparse row/col-blocks on col/row-major sparse matrices
		bRows = ".m_blockRows" if self.is_sparse else ".m_rows"
		bCols = ".m_blockCols" if self.is_sparse else ".m_cols"
		if self.rows <= 0 and self.valobj.GetValueForExpressionPath(bRows + ".m_value").IsValid():
			self.rows = self.valobj.GetValueForExpressionPath(bRows + ".m_value").GetValueAsSigned()
		if self.cols <= 0 and self.valobj.GetValueForExpressionPath(bCols + ".m_value").IsValid():
			self.cols = self.valobj.GetValueForExpressionPath(bCols + ".m_value").GetValueAsSigned()
		# For sparse col/row-blocks on col/row-major sparse matrices
		if self.rows <= 0: self.rows = self.expression.rows
		if self.cols <= 0: self.cols = self.expression.cols

		# Available on some dense and sparse blocks, but not needed it seems
		#outerStride = self.valobj.GetValueForExpressionPath(".m_outerStride")
		#innerStride = self.valobj.GetValueForExpressionPath(".m_innerStride")

		# Preload remaining properties
		self.size = self.rows*self.cols if self.rows*self.cols > 0 else -1
		if self.is_sparse:
			self.nnzs = 1 # TODO: Not Implemented Yet

		do_log(self.valobj, ("Dynamic " if self.size < 0 else "Static ") +
			("sparse " if self.is_sparse else "") + self.expression.elementType.GetName() +
			(" Block of size %d x %d" % (self.rows, self.cols)))

	def get_type_name(self):
		return self.valobj.GetType().GetName()

	def get_dimension_label(self):
		if self.size < 0: self.update()
		if self.size < 0: return "[invalid]"
		return "{%d,%d}" % (self.rows, self.cols)

	def has_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return False
		return self.expression.has_children()

	def num_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return 0
		return self.nnzs if self.is_sparse else self.size

	def get_element(self, row, col):
		if self.size < 0: return None
		return self.expression.get_element(self.startRow + row, self.startCol + col)

	def get_child_at_index(self, index):
		if self.size < 0: self.update()
		if self.size < 0: return None
		if index < 0: return None
		if index >= (self.nnzs if self.is_sparse else self.size): return None

		if self.is_sparse:
			# TODO: Not Implemented Yet
			return self.valobj.CreateValueFromExpression("[X]", '"Iteration Not Supported For Sparse Blocks"')
		else:
			# Parse as if indexed into continuous buffer [0-size]
			# This just affects display order, matching the order as if it was evaluated
			if self.expression.rowMajor: # RowMajor
				row = int(index / self.cols)
				col = int(index % self.cols)
			else: # Default, ColMajor
				row = int(index % self.rows)
				col = int(index / self.rows)
			return self.get_element(row, col)

class eigen_sparse_matrix:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.rowMajor = False
		self.values = self.outerIndices = self.innerIndices = self.innerNonZeros = None
		self.nnzs = self.outerSize = self.innerSize = self.rows = self.cols = 0
		self.size = -1
		self.elementType = None
		self.valid = get_toplevel_eigen_type(self.valobj) in ["SparseMatrix", "SparseVector"]

	def update(self):
		if not self.valid: return
		self.size = -1

		# Get typed layout information
		tgt = self.valobj.GetTarget()
		t = get_clean_type(self.valobj.GetType())
		try:
			opts = int(t.GetTemplateArgumentValue(tgt, 1).GetValue())
		except:
			do_log(self.valobj, "Failed to parse SparseMatrix template parameters!")
			debug_templates(self.valobj, t, tgt)
			return
		self.rowMajor = opts & 1

		# Get dynamic layout information
		try:
			if self.valobj.GetValueForExpressionPath(".m_size").IsValid(): # SparseVector
				self.outerSize = 1
				self.innerSize = self.valobj.GetValueForExpressionPath(".m_size").GetValueAsSigned()
				self.innerNonZeros = None
				self.outerIndices = None
			else: # SparseMatrix
				self.outerSize = self.valobj.GetValueForExpressionPath(".m_outerSize").GetValueAsSigned()
				self.innerSize = self.valobj.GetValueForExpressionPath(".m_innerSize").GetValueAsSigned()
				self.innerNonZeros = self.valobj.GetValueForExpressionPath(".m_innerNonZeros")
				if not self.innerNonZeros.IsValid() or self.innerNonZeros.GetValueAsUnsigned() == 0:
					self.innerNonZeros = None
				self.outerIndices = self.valobj.GetValueForExpressionPath(".m_outerIndex")
			self.innerIndices = self.valobj.GetValueForExpressionPath(".m_data.m_indices")
			self.values = self.valobj.GetValueForExpressionPath(".m_data.m_values")

			if self.outerIndices is None: # SparseVector
				self.nnzs = self.valobj.GetValueForExpressionPath(".m_data.m_size").GetValueAsSigned()
			elif self.outerSize == 0:
				self.nnzs = 0
			elif self.innerNonZeros is None: # Compressed
				outerBegin = self.outerIndices.GetChildAtIndex(0, lldb.eNoDynamicValues, True).GetValueAsSigned()
				outerEnd = self.outerIndices.GetChildAtIndex(self.outerSize, lldb.eNoDynamicValues, True).GetValueAsSigned()
				self.nnzs = outerEnd - outerBegin
			else:
				self.nnzs = 0
				for i in range(self.outerSize):
					self.nnzs += self.innerNonZeros.GetChildAtIndex(i, lldb.eNoDynamicValues, True).GetValueAsSigned()
		except:
			do_log(self.valobj, "Failed to evaluate SparseMatrix layout!")
			return

		# Preload remaining properties
		self.rows = self.outerSize if self.rowMajor else self.innerSize
		self.cols = self.innerSize if self.rowMajor else self.outerSize
		self.size = self.innerSize * self.outerSize
		self.elementType = self.values.GetType().GetPointeeType().GetCanonicalType()

		if self.outerIndices is None:
			do_log(self.valobj, self.elementType.GetName() +
				"SparseVector of size %d x %d with %d Non-Zeros" % (self.rows, self.cols, self.nnzs))
		else:
			do_log(self.valobj,
				("Compressed " if self.innerNonZeros is None else "Uncompressed ") +
				self.elementType.GetName() +
				(" SparseMatrix of size %d x %d with %d Non-Zeros" % (self.rows, self.cols, self.nnzs))
				+ (", row-major" if self.rowMajor else ""))

	def get_type_name(self):
		return self.valobj.GetType().GetName()

	def get_dimension_label(self):
		if self.size < 0: self.update()
		if self.size < 0: return "[invalid]"
		typeSign = ("V" if self.outerIndices is None else ("C" if self.innerNonZeros is None else "U"))
		return "[%d,%d](%d%s)" % (self.rows, self.cols, self.nnzs, typeSign)

	def has_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return False
		return self.nnzs > 0

	def num_children(self):
		if self.size < 0: self.update()
		if self.size < 0: return 0
		return self.nnzs

	def get_element(self, row, col):
		outerIndex = row if self.rowMajor else col
		innerIndex = col if self.rowMajor else row

		if self.outerIndices is not None: # SparseMatrix
			outerStart = self.outerIndices.GetChildAtIndex(outerIndex, lldb.eNoDynamicValues, True).GetValueAsSigned()
			outerEnd = self.outerIndices.GetChildAtIndex(outerIndex+1, lldb.eNoDynamicValues, True).GetValueAsSigned()
			if outerStart == outerEnd: # No element with given outerIndex
				return None

			if self.innerNonZeros is not None: # Uncompressed
				# May have masked elements at end of outerIndex values, fetch real NNZ count
				nnzs = self.innerNonZeros.GetChildAtIndex(outerIndex, lldb.eNoDynamicValues, True).GetValueAsSigned()
				outerEnd = outerStart+nnzs
		else: # SparseVector
			outerStart = 0
			outerEnd = self.nnzs

		for i in range(outerStart, outerEnd):
			index = self.innerIndices.GetChildAtIndex(i, lldb.eNoDynamicValues, True).GetValueAsSigned()
			if index == innerIndex:
				return self.values.GetChildAtIndex(i, lldb.eNoDynamicValues, True)
			if index > innerIndex:
				break

		# Element does not exist
		return None

	def get_child_at_index(self, nnz):
		if self.size < 0: self.update()
		if self.size < 0: return None
		if nnz < 0: return None
		if nnz >= self.nnzs: return None

		if self.innerNonZeros is not None: # Uncompressed
			# Find outerIndex first from NNZ index
			outerIndex = 0
			innerOffset = 0
			NNZs = 0
			for i in range(self.outerSize):
				innerNNZs = self.innerNonZeros.GetChildAtIndex(i, lldb.eNoDynamicValues, True).GetValueAsSigned()
				NNZs += innerNNZs
				if NNZs > nnz: # Desired NNZ is at outerIndex, and offset is also known
					outerIndex = i
					innerOffset = nnz - (NNZs-innerNNZs)
					break
			if NNZs < nnz:
				do_log(self.valobj, "Could not find NNZ %d/%d, max %d, is innerNonZeros corrupted?" % (nnz, self.nnzs, nnzs))
				return None

			# Fetch first value at this outerIndex
			outerStart = self.outerIndices.GetChildAtIndex(outerIndex, lldb.eNoDynamicValues, True).GetValueAsSigned()

			# Calculate real index with start of outerIndex and offset found before
			index = outerStart + innerOffset

		else: # Compressed
			index = nnz

			# Find outerIndex in index
			outerIndex = 0
			if self.outerIndices is not None:
				outerStart = self.outerIndices.GetChildAtIndex(0, lldb.eNoDynamicValues, True).GetValueAsSigned()
				for i in range(self.outerSize):
					outerEnd = self.outerIndices.GetChildAtIndex(i+1, lldb.eNoDynamicValues, True).GetValueAsSigned()
					if outerStart <= index < outerEnd:
						outerIndex = i
						break
					outerStart = outerEnd

		# Read value and innerIndex of Non-Zero Element with index
		element = self.values.GetChildAtIndex(index, lldb.eNoDynamicValues, True)
		innerIndex = self.innerIndices.GetChildAtIndex(index, lldb.eNoDynamicValues, True).GetValueAsSigned()

		if self.outerIndices is not None:
			row = outerIndex if self.rowMajor else innerIndex
			col = innerIndex if self.rowMajor else outerIndex
			return self.valobj.CreateValueFromData("[%d,%d]" % (row, col), element.GetData(), element.GetType())
		else:
			return self.valobj.CreateValueFromData("[%d]" % (innerIndex), element.GetData(), element.GetType())

def format_eigen_dense(valobj, matrix):
	if not matrix.valid:
		t = get_clean_type(valobj.GetType())
		do_log(valobj, "Falsely matched summary for invalid type %s" % (t.GetName()))
		# This happens if the summary was falsely matched by the regex against a type that the matrix parser then rejected
		return "" # Nothing we can do here to tell LLDB to move on with the original summary

	# Print matrix dimensions
	output = matrix.get_dimension_label()

	# Test if matrix is initialised
	if not matrix.has_children():
		return output + "[inaccessible]"

	# Inline matrices and vectors, small fully, large partially
	abortAtLen = 100 if matrix.size <= 9 else 50
	output += "[ "
	for row in range(0, matrix.rows):
		for col in range(0, matrix.cols):
			el = matrix.get_element(row, col)
			output += ("X" if el is None or not el.IsValid() else el.GetValue()) + " "
			if len(output) >= abortAtLen: break
		if col == matrix.cols-1:
			output = output[:-1] + ", "
		if len(output) >= abortAtLen: break
	if len(output) >= abortAtLen:
		output += "... ]"
	else:
		output = output[:-2] + " ]"
	do_log(valobj, "Inlined as " + output)
	return output

def format_eigen_dense_matrix_array_ref(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	dense = eigen_dense_matrix_array_ref(valobj, internal_dict)
	return format_eigen_dense(valobj, dense)

def format_eigen_dense_block(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	block = eigen_dense_block(valobj, internal_dict)
	return format_eigen_dense(valobj, block)

def format_eigen_sparse_matrix(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	sparse = eigen_sparse_matrix(valobj, internal_dict)
	return format_eigen_dense(valobj, sparse)

def __lldb_init_module (debugger, internal_dict):
	#lldb.formatters.Logger._lldb_formatters_debug_level = 2

	# Manually building a SBTypeCategory is a bad idea as SBTypeNameSpecifier does not match against CanonicalType
	# Meaning it can't match against e.g. Eigen::Block because that is usually a ::FixedBlockXpr when not fully resolved
	# This command line however WILL match against the underlying canonical type
	# While these regex matches are quite strict, there may still be false matches
	# Synthetic can handle that by just returning no children with valid = False
	# Summary however can not opt-out, LLDB will use the first matched summary
	debugger.HandleCommand("type summary add -x \"^Eigen::Matrix<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type summary add -x \"^Eigen::Array<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type summary add -x \"^Eigen::Block<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_dense_block")
	debugger.HandleCommand("type summary add -x \"^Eigen::Ref<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type summary add -x \"^Eigen::SparseMatrix<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_sparse_matrix")
	debugger.HandleCommand("type summary add -x \"^Eigen::SparseVector<.+>$\" -F LLDB_Eigen_Data_Formatter.format_eigen_sparse_matrix")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::Matrix<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::Array<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::Block<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_dense_block")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::Ref<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_dense_matrix_array_ref")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::SparseMatrix<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_sparse_matrix")
	debugger.HandleCommand("type synthetic add -x \"^Eigen::SparseVector<.+>$\" -l LLDB_Eigen_Data_Formatter.eigen_sparse_matrix")
