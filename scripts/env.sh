# source this before building. Caps parallel jobs for low-RAM machines.
# On the author's machine pixi lives in ~/workspace/.tools; elsewhere the normal pixi install is used.
if [ -d "$HOME/workspace/.tools/pixi" ]; then
  export PIXI_HOME=$HOME/workspace/.tools/pixi
  export PIXI_CACHE_DIR=$HOME/workspace/.tools/pixi-cache
  export RATTLER_CACHE_DIR=$HOME/workspace/.tools/pixi-cache
  export PATH=$HOME/workspace/.tools/bin:$PATH
fi
export MAKEFLAGS=-j2
