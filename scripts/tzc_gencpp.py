import genmsg.msgs
import gencpp

MSG_TYPE_TO_CPP = {
  'byte': 'int8_t',
  'char': 'uint8_t',
  'bool': 'uint8_t',
  'uint8': 'uint8_t',
  'int8': 'int8_t',
  'uint16': 'uint16_t',
  'int16': 'int16_t',
  'uint32': 'uint32_t',
  'int32': 'int32_t',
  'uint64': 'uint64_t',
   'int64': 'int64_t',
  'float32': 'float',
  'float64': 'double',
  'string': 'std::string',
  'time': 'ros::Time',
  'duration': 'ros::Duration'
}

def is_fixed_length(type, package, msg_context, search_path):
  if (type == 'string'):
    return False
  if (genmsg.msgs.is_builtin(type)):
    return True
  (base_type, is_array, array_len) = genmsg.msgs.parse_type(type)
  if (not is_array):
    rtype = genmsg.msgs.resolve_type(type, package)
    spec = genmsg.msg_loader.load_msg_by_type(msg_context, rtype, search_path)
    return gencpp.is_fixed_length(spec, msg_context, search_path)
  if (not array_len is None):
    return False
  return is_fixed_length(base_type, package, msg_context, search_path)

def use_tzc_type(type, package, msg_context, search_path):
  type = genmsg.msgs.resolve_type(type, package)
  spec = genmsg.msg_loader.load_msg_by_type(msg_context, type, search_path)
  for field in spec.parsed_fields():
    if (not field.is_array):
      if (not field.is_builtin):
        if (use_tzc_type(field.type, spec.package, msg_context, search_path)):
          return True
    elif (is_fixed_length(field.base_type, spec.package, msg_context, search_path)):
      if (field.array_len is None):
        return True
    elif (field.base_type == 'string'):
      continue
    else:
      if (use_tzc_type(field.base_type, spec.package, msg_context, search_path)):
        return True
  return False

def msg_type_to_cpp(type, package, msg_context, search_path):
  (base_type, is_array, array_len) = genmsg.msgs.parse_type(type)
  cpp_type = None
  is_builtin = genmsg.msgs.is_builtin(base_type)
  if (is_builtin):
    cpp_type = MSG_TYPE_TO_CPP[base_type]
  elif (len(base_type.split('/')) == 1):
    if (genmsg.msgs.is_header_type(base_type)):
      cpp_type = '::std_msgs::Header'
    elif (use_tzc_type(base_type, package, msg_context, search_path)):
      cpp_type = '::tzc_transport::%s::%s_'%(package, base_type)
    else:
      cpp_type = '%s'%(base_type)
  else:
    pkg = base_type.split('/')[0]
    msg = base_type.split('/')[1]
    if (use_tzc_type(base_type, package, msg_context, search_path)):
      cpp_type = '::tzc_transport::%s::%s_'%(pkg, msg)
    else:
      cpp_type = '::%s::%s'%(pkg, msg)
  if (not is_array):
    return cpp_type
  if (not array_len is None):
    return 'boost::array< %s, %s >'%(cpp_type, array_len)
  if (is_fixed_length(base_type, package, msg_context, search_path)):
    return 'vector< %s >'%(cpp_type) # tzc_transport::vector
  return 'std::vector< %s >'%(cpp_type)

def spec_to_getlength(spec, msg_context, search_path, prefix, pre_line):
  for field in spec.parsed_fields():
    fullname = prefix + field.name
    itername = '_iter_' + field.name
    if (not field.is_array):
      if (not field.is_builtin):
        rtype = genmsg.msgs.resolve_type(field.type, spec.package)
        nspec = genmsg.msg_loader.load_msg_by_type(msg_context, rtype, search_path)
        spec_to_getlength(nspec, msg_context, search_path, fullname + '.', pre_line);
    elif (is_fixed_length(field.base_type, spec.package, msg_context, search_path)):
      if (field.array_len is None):
        cpp_type = msg_type_to_cpp(field.base_type, spec.package, msg_context, search_path)
        print('%sres += sizeof(%s) * %s.size_;'%(pre_line, cpp_type, fullname))
    elif (field.base_type == 'string'):
      continue
    else:
      rtype = genmsg.msgs.resolve_type(field.base_type, spec.package)
      nspec = genmsg.msg_loader.load_msg_by_type(msg_context, rtype, search_path)
      print('%sfor (auto & %s : %s) {'%(pre_line, itername, fullname))
      spec_to_getlength(nspec, msg_context, search_path, itername + '.', pre_line + '  ');
      print('%s}'%(pre_line))

def spec_to_fillarray(spec, msg_context, search_path, prefix, pre_line):
  for field in spec.parsed_fields():
    fullname = prefix + field.name
    itername = '_iter_' + field.name
    if (not field.is_array):
      if (not field.is_builtin):
        rtype = genmsg.msgs.resolve_type(field.type, spec.package)
        nspec = genmsg.msg_loader.load_msg_by_type(msg_context, rtype, search_path)
        spec_to_fillarray(nspec, msg_context, search_path, fullname + '.', pre_line);
    elif (is_fixed_length(field.base_type, spec.package, msg_context, search_path)):
      if (field.array_len is None):
        cpp_type = msg_type_to_cpp(field.base_type, spec.package, msg_context, search_path)
        print('%s%s.ptr_ = (%s *)tmp;'%(pre_line, fullname, cpp_type))
        print('%stmp = (void *)(%s.ptr_ + %s.size_);'%(pre_line, fullname, fullname))
    elif (field.base_type == 'string'):
      continue
    else:
      rtype = genmsg.msgs.resolve_type(field.base_type, spec.package)
      nspec = genmsg.msg_loader.load_msg_by_type(msg_context, rtype, search_path)
      print('%sfor (auto & %s : %s) {'%(pre_line, itername, fullname))
      spec_to_fillarray(nspec, msg_context, search_path, itername + '.', pre_line + '  ');
      print('%s}'%(pre_line))

