import sys
import os

INIT_FILE_NAME = "workspace.init"
FUNCTIONS = ['deactivate','_rosws_install_deps','rosws','catkin','catkin_make']


def echo(args):
  print "echo \""+args+"\""

def export(name,value):
  print "export %s=\"%s\"" % (name, value)

def source(filename):
  print "source %s" % filename

def unset(name):
  print "unset %s" % name

def active():
  if "ROS_WORKSPACE" in os.environ:
    return True

def get_current_workspace():
  if active():
    return os.environ["ROS_WORKON_WORKSPACE"]
  return None

def get_current_workspace_dir():
  if active():
    return os.environ["ROS_WORKSPACE"]
  return None

def get_workspaces(root):
  workspaces = ( d for d in os.listdir(root) if os.path.isdir(os.path.join(root,d)) )
  return ( w for w in workspaces if validate_workspace(w,root=root) )

def print_workspaces(root):
  for ws in get_workspaces(root):
    echo(ws)

def validate_workspace(workspace,root):
  # check if this is a workspace
  # should have a devel dir, src dir
  # devel dir should have a setup.bash
  for f in (".catkin_workspace",".catkin_tools","devel/setup.bash"):
    if os.path.exists(os.path.join(root, workspace, ".catkin_workspace")):
      return True
  return False

def activate_workspace(workspace,root):
  if validate_workspace(workspace,root=root):
    source(os.path.join(root, workspace, "devel", "setup.bash"))

    # Source the workspace's init file if it exists
    initfile = os.path.join(root, workspace, INIT_FILE_NAME)
    if os.path.exists(initfile):
      source(initfile)

    print "export ROS_WORKSPACE=%s" % os.path.join(root,workspace)
    print "export ROS_WORKON_WORKSPACE=%s" % workspace

    print "catkin_make()"
    print "{"
    print "  CATKIN_MAKE=`which catkin_make`"
    print "  (cd $ROS_WORKSPACE ; $CATKIN_MAKE \"$@\")"
    print "}"

    print "catkin()"
    print "{"
    print " CATKIN=`which catkin`"
    print " if [ \"$1\" == \"list\" ]; then"
    print "   ( $CATKIN \"$@\" )"
    print " else"
    print "   VERB=\"$1\""
    print "   shift"
    print "   $CATKIN $VERB -w $ROS_WORKSPACE \"$@\""
    print " fi"
    print "}"

    print "_rosworkon_install_deps()"
    print "{"
    print "  (cd $ROS_WORKSPACE ; rosdep install --from-paths src -i -y -r)"
    print "}"

    print "rosws()"
    print "{"
    print "  wstool $@"
    print "}"

    print "deactivate()"
    print "{"
    print "rosworkon --deactivate"
    print "}"
  else:
    echo("Invalid workspace: %s" % workspace)

def deactivate():
  for v in os.environ.keys():
    if v.startswith("ROS_") and v != "ROS_WORKON_ROOT":
      unset(v)
  unset("ROSLISP_PACKAGE_DIRECTORIES")
  for f in FUNCTIONS:
    print "unset -f %s" % f

def init():
  # Setup tab completion
  print "_rosworkon_list()"
  print "{"
  print '  local cur="${COMP_WORDS[COMP_CWORD]}"'
  print 'COMPREPLY=( $(compgen -W "`eval \"$(python %s --autocomplete ${cur})\"`" -- ${cur})  )' % os.path.realpath(__file__)
  print "}"
  print "complete -o default -o nospace -F _rosworkon_list rosworkon"

def print_options():
  echo("--init")
  echo("--deactivate")
  echo("--install-deps")

def print_current_workspace():
  echo("Current workspace: %s (%s)" % (get_current_workspace(),get_current_workspace_dir()))

def main():
  if not "ROS_WORKON_ROOT" in os.environ:
    echo("ROS_WORKON_ROOT not set. Please set it to use rosworkon.")
    return

  WORKSPACES_ROOT = os.environ['ROS_WORKON_ROOT']

  if len(sys.argv)==1:
    if active():
      print_current_workspace()
    echo("Available workspaces:")
    print_workspaces(root=WORKSPACES_ROOT)
  elif len(sys.argv)>1:
    if(sys.argv[1]=="--deactivate"):
      deactivate()
    elif(sys.argv[1]=="--install-deps"):
      print "_rosworkon_install_deps"
    elif(sys.argv[1]=="--autocomplete"):
      print_workspaces(root=WORKSPACES_ROOT)
      if len(sys.argv) > 2 and sys.argv[2][0] == '-':
        print_options()
    elif(sys.argv[1]=="--init"):
      init()
    else:
      activate_workspace(sys.argv[1], root=WORKSPACES_ROOT)

if __name__ == "__main__":
  main()
