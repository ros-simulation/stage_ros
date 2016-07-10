#!/usr/bin/env python
import rospy
import sys

def instantiate_rosparams(world_str_in, rosparam_objs):
	"""
	Include this function if you want to embed this script's functionality in your own script.
	"""
	world_str_out = world_str_in + '\n\n# Additional rosparam objects\n\n'

	for obj_name, obj_props in rosparam_objs.items():
		try:
			obj_str = obj_props['model_type'] + '\n(\n'
			obj_str += '    name "%s"\n' % str(obj_name)
			for prop, val in obj_props.items():
				if str(prop) == 'model_type': continue
				obj_str += '    %s ' % str(prop)
				if isinstance(val, int) or isinstance(val, float):
					obj_str += str(val)
				elif isinstance(val, list):
					obj_str += '[ %s ]' % ' '.join(map(str, val))
				else:
					obj_str += '"%s"' % str(val)
				obj_str += '\n'
			obj_str += ')\n\n'
			world_str_out += obj_str

		except Exception as e:
			rospy.logwarn('Skip inserting "%s" into stage world:\n%s' % (str(obj_name), str(e)))

	return world_str_out


if __name__ == '__main__':

	rospy.init_node('stage_rosparam_instantiate')

	if len(sys.argv) != 4:
		rospy.logwarn('''Failed to instantiate objects in stage world: Wrong number of arguments!
Usage: rosparam_instantiate.py worldfile_in rosparam_ns worldfile_out
 - worldfile_in:  Path to initial .world file.
 - rosparam_ns:   Namespace which contains the objects to be instantiated.
 - worldfile_out: Path to resulting .world file (loaded by stage).''')

	else:
		worldfile_in = sys.argv[1]
		rosparam_ns = sys.argv[2]
		worldfile_out = sys.argv[-1]

		world_str_out = None
		try:
			world_str_in = ""
			with open(worldfile_in, 'r') as f:
				world_str_in = f.read()

			rosparam_objs = rospy.get_param(rosparam_ns)
			if not isinstance(rosparam_objs, dict):
				raise TypeError(rosparam_objs.__class__)

			# do the work...
			world_str_out = instantiate_rosparams(world_str_in, rosparam_objs)

		except IOError as e:
			rospy.logerr('Unable to load initial .world file for stage:\n%s' % str(e))
		except KeyError as e:
			rospy.logerr('Unable to access rosparam for stage:\nMissing key: %s' % str(e))
		except TypeError as e:
			rospy.logerr("Type mismatch of rosparam for stage:\nParam '%s' is %s, but should be a dictionary/namespace." % (rosparam_ns, str(e)))

		if world_str_out is not None:
			header_label = '# AUTO-GENERATED FILE by rosparam_instantiate.py'
			header_args = '# args: ' + ' '.join(sys.argv[1:])
			header_border = '#' * (max(len(header_label), len(header_args)) + 1)
			header = '\n'.join([header_border, header_label, header_args, header_border, '\n'])

			try:
				with open(worldfile_out, 'w') as f:
					f.write(header)
					f.write(world_str_out)

			except IOError as e:
				rospy.logerr('Unable to write resulting .world file for stage:\n%s' % str(e))
