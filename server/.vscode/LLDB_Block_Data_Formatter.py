'''
Copyright © 2026 Seneral

Distributed under the GNU GENERAL PUBLIC LICENSE VERSION 3.0.

For custom BlockedVector and BlockedQueue data structures.
'''

import lldb
import os
import re
import pdb
import traceback
from inspect import getmembers, isfunction

def get_expression_path(valobj):
	# Determine expression path of the valobj
	stream = lldb.SBStream()
	valobj.GetExpressionPath(stream)
	return stream.GetData()

def do_log(valobj, msg):
	# Log with context of object
	lldb.formatters.Logger.Logger() >> "'" + get_expression_path(valobj) + "': " + msg

def debug_templates(valobj, t, tgt):
	lldb.formatters.Logger.Logger() >> "Type %s has %d template arguments:" % (t.GetName(), t.GetNumberOfTemplateArguments())
	try:
		for i in range(t.GetNumberOfTemplateArguments()):
			if t.GetTemplateArgumentKind(i) == lldb.eTemplateArgumentKindType:
				lldb.formatters.Logger.Logger() >> "Template parameter %d: type %s" % (i, t.GetTemplateArgumentType(i).GetName())
			else:
				lldb.formatters.Logger.Logger() >> "Template parameter %d: kind %d, value %s" % (i, t.GetTemplateArgumentKind(i), str(t.GetTemplateArgumentValue(tgt, i).GetValue()))
	except:
		lldb.formatters.Logger.Logger() >> "Iterating template arguments failed: %s" % (traceback.format_exc())

def get_clean_type(t):
	t = t.GetUnqualifiedType().GetCanonicalType()
	if t.IsReferenceType(): t = t.GetDereferencedType()
	if t.IsPointerType(): t = t.GetPointeeType()
	return t

class blocked_queue:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.base = None
		self.blockStart = self.blockCount = self.blockSize = -1
		self.startIndex = self.endIndex = self.size = -1

		# Get typed layout information
		t = get_clean_type(self.valobj.GetType())
		try: # Strangely, the integral Block Size template value is not accessible at all via t.GetTemplateArgumentValue(tgt, 1), likely LLDB regression
			queue = re.match("^BlockedQueue<.+, ?([0-9]+)U?L?>$", t.GetName())
			if queue: self.blockSize = int(queue.group(1))
		except:
			do_log(self.valobj, "Failed to parse template parameters of %s (num %d): %s" % (t.GetName(), t.GetNumberOfTemplateArguments(), traceback.format_exc()))

	def update(self):
		state = self.valobj.GetValueForExpressionPath(".m_state")
		self.blockStart = state.GetValueForExpressionPath(".start").GetValueAsUnsigned()
		self.blockCount = state.GetValueForExpressionPath(".count").GetValueAsUnsigned()
		self.startIndex = self.blockStart * self.blockSize
		self.endIndex = state.GetValueForExpressionPath(".index").GetValueAsUnsigned()
		self.size = self.endIndex - self.startIndex

		self.base = self.valobj.GetValueForExpressionPath(".m_base")

		do_log(self.valobj, "BlockedQueue<%d> %s of size %d (%d -> %d)" % (self.blockSize, self.valobj.GetName(), self.size, self.startIndex, self.endIndex))

	def has_children(self):
		if self.size < 0: self.update()
		return self.size > 0

	def num_children(self):
		if self.size < 0: self.update()
		return self.size

	def get_element(self, block, index):
		if self.size < 0: return None
		frame = self.valobj.GetFrame()
		try:
			if block == 0:
				element = frame.EvaluateExpression("(*%s.begin().operator->())[%d]" % (get_expression_path(self.base), index))
				return element

			if block < self.blockCount/2:
				blockIt = frame.EvaluateExpression("%s.begin()" % (get_expression_path(self.base)))
				for b in range(0, block):
					blockIt = frame.EvaluateExpression("++" + blockIt.GetName())
			else:
				blockIt = frame.EvaluateExpression("--%s.end()" % (get_expression_path(self.base)))
				for b in range(block, self.blockCount-1):
					blockIt = frame.EvaluateExpression("--" + blockIt.GetName())

			return frame.EvaluateExpression("(*%s.operator->())[%d]" % (blockIt.GetName(), index))
		except:
			do_log(self.valobj, "Failed to get element: %s" % (traceback.format_exc()))
			return None

	def get_child_at_index(self, index):
		if self.size < 0: self.update()
		if self.size < 0: return None
		if index < 0 or index >= self.size: return None

		# Show back-to-front
		index = self.size-1-index

		element = self.get_element(int(index / self.blockSize) + self.blockStart, index % self.blockSize)
		return self.valobj.CreateValueFromData("[%d]" % (self.startIndex + index), element.GetData(), element.GetType())

class blocked_queue_view:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.blockStart = self.blockCount = self.blockSize = -1
		self.startIndex = self.endIndex = self.size = -1

		# Get typed layout information
		t = get_clean_type(self.valobj.GetType())
		try:
			view = re.match("^BlockedQueue<.+, ?([0-9]+)U?L?>::View<.+>$", t.GetName())
			self.blockSize = int(view.group(1))
		except:
			do_log(self.valobj, "Failed to parse template parameters of %s (num %d): %s" % (t.GetName(), t.GetNumberOfTemplateArguments(), traceback.format_exc()))

	def update(self):
		state = self.valobj.GetValueForExpressionPath(".m_state")
		self.blockStart = state.GetValueForExpressionPath(".start").GetValueAsUnsigned()
		self.blockCount = state.GetValueForExpressionPath(".count").GetValueAsUnsigned()
		self.startIndex = self.blockStart * self.blockSize
		self.endIndex = state.GetValueForExpressionPath(".index").GetValueAsUnsigned()
		self.size = self.endIndex - self.startIndex

		do_log(self.valobj, "BlockedQueue<%d>::View %s of size %d (%d -> %d)" % (self.blockSize, self.valobj.GetName(), self.size, self.startIndex, self.endIndex))

	def has_children(self):
		if self.size < 0: self.update()
		return self.size > 0

	def num_children(self):
		if self.size < 0: self.update()
		return self.size

	def get_child_at_index(self, index):
		if self.size < 0: self.update()
		if self.size < 0: return None
		if index < 0 or index >= self.size: return None
	
		# Show back-to-front
		index = self.size-1-index

		element = self.valobj.EvaluateExpression("operator[](%d)" % (self.startIndex + index))
		return self.valobj.CreateValueFromData("[%d]" % (self.startIndex + index), element.GetData(), element.GetType())

class blocked_iterator:
	def __init__(self, valobj, internal_dict):
		self.valobj = valobj
		self.element = None
		self.index = -1
		t = get_clean_type(self.valobj.GetType())
		self.is_vector = t.GetName().startswith("BlockedVector")

	def update(self):
		self.index = self.valobj.EvaluateExpression("index()").GetValueAsUnsigned()
		try:
			check = "valid()" if self.is_vector else "accessible()"
			if self.valobj.EvaluateExpression(check).GetValueAsUnsigned() > 0:
				element = self.valobj.EvaluateExpression("operator*()")
				self.element = self.valobj.CreateValueFromData("[%d]" % (self.index), element.GetData(), get_clean_type(element.GetType()))
		except:
			do_log(self.valobj, "Failed to evaluate BlockedQueue iterator: " + traceback.format_exc())

	def get_type_name(self):
		return self.valobj.GetType().GetName()

	def has_children(self):
		if self.index < 0: self.update()
		return self.element is not None and self.element.num_children > 0

	def num_children(self):
		if self.index < 0: self.update()
		return 0 if self.element is None else self.element.num_children

	def get_child_at_index(self, index):
		if self.index < 0: self.update()
		return None if self.element is None else self.element.GetChildAtIndex(index)

def format_blocked_container(valobj, container):
	container.update()
	output = "[%d]{%d,%d} " % (container.size, container.startIndex, container.endIndex)
	if isinstance(container, blocked_queue):
		output += "BlockedQueue "
	elif isinstance(container, blocked_queue_view):
		output += "BlockedQueue::View "
	#elif isinstance(container, blocked_vector):
	#	output += "BlockedVector "

	try:
		if not container.has_children():
			return output + "[inaccessible]"

		# Inline matrices and vectors, small fully, large partially
		abortAtLen = 50
		output += "[ "
		for index in range(0, container.size):
			el = container.get_child_at_index(index)
			output += ("X" if el is None or not el.IsValid() else str(el.GetValue())) + " "
			if len(output) >= abortAtLen: break
		if len(output) >= abortAtLen:
			output += "... ]"
		else:
			output += "]"
		do_log(valobj, "Inlined as " + output)
		return output
	except:
		do_log(valobj, "Failed to iterate summary of blocked container: " + traceback.format_exc())
		return output

""" def format_blocked_vector(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	return format_blocked_container(valobj, blocked_vector(valobj, internal_dict)) """

def format_blocked_queue(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	return format_blocked_container(valobj, blocked_queue(valobj, internal_dict))

def format_blocked_queue_view(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	return format_blocked_container(valobj, blocked_queue_view(valobj, internal_dict))

def format_blocked_iterator(valobj, internal_dict):
	# Get original value, not copy with synthetic children
	valobj = valobj.GetNonSyntheticValue() if valobj.IsSynthetic() else valobj
	try:
		iterator = blocked_iterator(valobj, internal_dict)
		iterator.update()
		if iterator.element is None:
			return "[%d] [inaccessible]" % (iterator.index)
		return "[%d] %s" % (iterator.index, str(iterator.element))
	except:
		do_log(valobj, "format_blocked_iterator failed: " + traceback.format_exc())
		return "[inaccessible]"

def __lldb_init_module (debugger, internal_dict):
	lldb.formatters.Logger._lldb_formatters_debug_level = 2

	# Manually building a SBTypeCategory is a bad idea as SBTypeNameSpecifier does not match against CanonicalType
	# Meaning it can't match against e.g. Eigen::Block because that is usually a ::FixedBlockXpr when not fully resolved
	# This command line however WILL match against the underlying canonical type
	# While these regex matches are quite strict, there may still be false matches
	# Synthetic can handle that by just returning no children with valid = False
	# Summary however can not opt-out, LLDB will use the first matched summary

	#debugger.HandleCommand("type summary add -x \"^BlockedVector<.+, ?[0-9]+U?L?>$\" -F LLDB_Block_Data_Formatter.format_blocked_vector")
	debugger.HandleCommand("type summary add -x \"^BlockedQueue<.+, ?[0-9]+U?L?>$\" -F LLDB_Block_Data_Formatter.format_blocked_queue")
	debugger.HandleCommand("type summary add -x \"^BlockedQueue<.+>::View<.+>$\" -l LLDB_Block_Data_Formatter.format_blocked_queue_view")

	debugger.HandleCommand("type summary add -x \"^BlockedVector<.+>::iterator_t<.+>$\" -F LLDB_Block_Data_Formatter.format_blocked_iterator")
	debugger.HandleCommand("type summary add -x \"^BlockedQueue<.+>::iterator_t<.+>$\" -F LLDB_Block_Data_Formatter.format_blocked_iterator")

	#debugger.HandleCommand("type synthetic add -x \"^BlockedVector<.+, ?[0-9]+U?L?>$\" -l LLDB_Block_Data_Formatter.blocked_vector")
	debugger.HandleCommand("type synthetic add -x \"^BlockedQueue<.+, ?[0-9]+U?L?>$\" -l LLDB_Block_Data_Formatter.blocked_queue")
	debugger.HandleCommand("type synthetic add -x \"^BlockedQueue<.+>::View<.+>$\" -l LLDB_Block_Data_Formatter.blocked_queue_view")

	debugger.HandleCommand("type synthetic add -x \"^BlockedVector<.+>::iterator_t<.+>$\" -l LLDB_Block_Data_Formatter.blocked_iterator")
	debugger.HandleCommand("type synthetic add -x \"^BlockedQueue<.+>::iterator_t<.+>$\" -l LLDB_Block_Data_Formatter.blocked_iterator")
