#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

PROJECT_ROOT="${PIXI_PROJECT_ROOT:-$(pwd)}"

# Per-machine settings: config/local.env (gitignored; the keys are in config/local.env.example).
# Plain KEY=value lines, read as data, not run as shell code. A variable already set in the
# environment wins over the file, so a single run can override it:
#     ROS_DOMAIN_ID=7 pixi run teleop
LOCAL_ENV_FILE="$PROJECT_ROOT/config/local.env"
if [ -f "$LOCAL_ENV_FILE" ]; then
    while IFS= read -r _line || [ -n "$_line" ]; do
        # Trim surrounding whitespace, skip blank lines and comments.
        _line="${_line#"${_line%%[![:space:]]*}"}"
        _line="${_line%"${_line##*[![:space:]]}"}"
        case "$_line" in ''|'#'*) continue ;; esac
        _key="${_line%%=*}"
        _value="${_line#*=}"
        case "$_value" in
            \"*\") _value="${_value#\"}"; _value="${_value%\"}" ;;
            \'*\') _value="${_value#\'}"; _value="${_value%\'}" ;;
        esac
        case "$_key" in
            "$_line"|''|[0-9]*|*[!A-Za-z0-9_]*) _bad=1 ;;
            *) case "$_value" in *'$'*|*'`'*) _bad=1 ;; *) _bad= ;; esac ;;
        esac
        if [ -n "$_bad" ]; then
            echo "WARNING: config/local.env: ignoring '$_line' (expected KEY=value, no \$ or \`)" >&2
            continue
        fi
        eval "_current=\${$_key:-}"
        [ -n "$_current" ] && continue
        # The profile path may be relative to the project root: Fast DDS would resolve it
        # against each process's working directory.
        case "$_key=$_value" in
            FASTRTPS_DEFAULT_PROFILES_FILE=[!/]*) _value="$PROJECT_ROOT/$_value" ;;
        esac
        export "$_key=$_value"
    done < "$LOCAL_ENV_FILE"
    unset _line _key _value _bad _current
fi

# Defaults for whatever the personal env file did not set. Deliberately NOT gated on
# $ROS_DISTRO: in the default pixi environment ROS_DISTRO is exported by a conda
# activation script that runs after this one, so such a guard silently skips this
# block. `pixi run robot` then sends `export ROS_DOMAIN_ID=` over ssh and the jetson
# cameras come up on domain 0 while every PC node runs on domain 5.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" #rmw_cyclonedds_cpp

# Fast DDS profile: none by default, so a machine runs on the Fast DDS defaults.
# config/fastdds_config.xml is the DFKI robot PC's (loopback + camera link only, unicast
# discovery for domain 5; see the comment in it). Enable it on that PC only, in
# config/local.env:
#     FASTRTPS_DEFAULT_PROFILES_FILE=config/fastdds_config.xml
case "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" in
    */config/fastdds_config.xml)
        # Its unicast discovery ports are derived from the domain id. On a mismatch the
        # later participants island silently, so say so loudly.
        if [ "$ROS_DOMAIN_ID" != "5" ]; then
            echo "WARNING: ROS_DOMAIN_ID=$ROS_DOMAIN_ID but config/fastdds_config.xml lists" >&2
            echo "         discovery ports for domain 5. Regenerate them or discovery will" >&2
            echo "         silently island nodes (see the comment in that file)." >&2
        fi
        ;;
esac

WORKSPACE_SETUP="$PROJECT_ROOT/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
fi

ros2 daemon stop
ros2 daemon start
