function matlab_ds_ros_bridge(varargin)
%MATLAB_DS_ROS_BRIDGE ROS2 bridge to run DS.m and publish Cartesian commands.
%
% Usage examples:
%   matlab_ds_ros_bridge()
%   matlab_ds_ros_bridge('AlgoName','SEDS','TrajectoryFile','demo_traj.mat')
%   matlab_ds_ros_bridge('AlgoName','LPVDS','DoPlots',false)
%
% Topics:
%   Subscribes : /rlearn/state           (std_msgs/Float64MultiArray)
%                [t, x, y, z, vx, vy, vz]
%                /controller/learn_status (std_msgs/String)
%                expects status 'recording_stopped:<path_to_dataset>'
%   Publishes  : /rlearn/command_twist   (std_msgs/Float64MultiArray)
%                [vx, vy, vz]
%
% Notes:
% - By default, this bridge uses SEDS if trajectories are provided.
% - If no trajectory file is provided, a linear DS is used as a safe fallback.

    cfg = parseInputs(varargin{:});

    bridge = struct();
    bridge.cfg = cfg;
    bridge.ds = createDsController(cfg);

    bridge.node = ros2node(cfg.NodeName);
    bridge.cmdPub = ros2publisher(bridge.node, cfg.CommandTopic, 'std_msgs/Float64MultiArray');

    bridge.stateSub = ros2subscriber( ...
        bridge.node, ...
        cfg.StateTopic, ...
        'std_msgs/Float64MultiArray', ...
        @(msg)stateCallback(msg, bridge), ...
        'History', 'keeplast', ...
        'Depth', 1);

    bridge.learnStatusSub = ros2subscriber( ...
        bridge.node, ...
        cfg.LearnStatusTopic, ...
        'std_msgs/String', ...
        @(msg)learnStatusCallback(msg, bridge), ...
        'History', 'keeplast', ...
        'Depth', 10);

    cleaner = onCleanup(@()shutdownBridge(bridge)); %#ok<NASGU>

    fprintf('[MATLAB DS Bridge] Node: %s\n', cfg.NodeName);
    fprintf('[MATLAB DS Bridge] Subscribing: %s\n', cfg.StateTopic);
    fprintf('[MATLAB DS Bridge] Subscribing: %s\n', cfg.LearnStatusTopic);
    fprintf('[MATLAB DS Bridge] Publishing : %s\n', cfg.CommandTopic);
    fprintf('[MATLAB DS Bridge] Algorithm  : %s\n', cfg.AlgoName);

    while true
        pause(0.1);
    end
end

function cfg = parseInputs(varargin)
    p = inputParser;
    addParameter(p, 'NodeName', '/matlab_ds_bridge', @(x)ischar(x) || isstring(x));
    addParameter(p, 'StateTopic', '/rlearn/state', @(x)ischar(x) || isstring(x));
    addParameter(p, 'CommandTopic', '/rlearn/command_twist', @(x)ischar(x) || isstring(x));
    addParameter(p, 'LearnStatusTopic', '/controller/learn_status', @(x)ischar(x) || isstring(x));
    addParameter(p, 'AlgoName', 'SEDS', @(x)ischar(x) || isstring(x));
    addParameter(p, 'TrajectoryFile', '', @(x)ischar(x) || isstring(x));
    addParameter(p, 'TrajectoryVariable', 'trajectories', @(x)ischar(x) || isstring(x));
    addParameter(p, 'DoPlots', false, @islogical);
    addParameter(p, 'DoModulation', false, @islogical);
    addParameter(p, 'MaxLinearSpeed', 0.25, @(x)isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Attractor', [0.23; 0.02; 0.28], @(x)isnumeric(x) && numel(x)==3);
    parse(p, varargin{:});

    cfg = p.Results;
    cfg.NodeName = char(cfg.NodeName);
    cfg.StateTopic = char(cfg.StateTopic);
    cfg.CommandTopic = char(cfg.CommandTopic);
    cfg.LearnStatusTopic = char(cfg.LearnStatusTopic);
    cfg.AlgoName = upper(char(cfg.AlgoName));
    cfg.TrajectoryFile = char(cfg.TrajectoryFile);
    cfg.TrajectoryVariable = char(cfg.TrajectoryVariable);
    cfg.Attractor = reshape(double(cfg.Attractor), [3, 1]);
end

function dsObj = createDsController(cfg)
    ensureDsLibraryPaths();

    dsObj = DS(cfg.AlgoName, [], cfg.DoPlots, cfg.DoModulation);

    if strlength(string(cfg.TrajectoryFile)) > 0 && isfile(cfg.TrajectoryFile)
        loaded = load(cfg.TrajectoryFile);
        if ~isfield(loaded, cfg.TrajectoryVariable)
            error('Trajectory variable "%s" not found in file "%s".', cfg.TrajectoryVariable, cfg.TrajectoryFile);
        end

        trajectories = loaded.(cfg.TrajectoryVariable);
        validateattributes(trajectories, {'numeric'}, {'nonempty', 'real', 'finite'});

        switch cfg.AlgoName
            case 'SEDS'
                dsObj.learnSEDS(trajectories);
            case 'LPVDS'
                dsObj.learnLPVDS(trajectories);
            otherwise
                warning('Unknown AlgoName "%s". Falling back to linear DS.', cfg.AlgoName);
                dsObj.setLinearDS(cfg.Attractor);
        end
    else
        warning(['No trajectory file provided/found. ' ...
                 'Using a linear DS fallback (setLinearDS).']);
        dsObj.setLinearDS(cfg.Attractor);
    end
end

function ensureDsLibraryPaths()
    persistent pathsInitialized;
    if ~isempty(pathsInitialized) && pathsInitialized
        return;
    end

    thisFile = mfilename('fullpath');
    if isempty(thisFile)
        projectRoot = pwd;
    else
        projectRoot = fileparts(thisFile);
    end

    % Remove known conflicting toolbox paths first (if present).
    conflicting = {
        fullfile(projectRoot, 'libraries', 'book-sods-opt')
    };
    for i = 1:numel(conflicting)
        p = conflicting{i};
        if isfolder(p)
            rmpath(genpath(p));
        end
    end

    % Add minimal required paths with explicit priority.
    highPriority = {
        fullfile(projectRoot, 'libraries', 'book-thirdparty', 'seds-stripped', 'SEDS_lib'), ...
        fullfile(projectRoot, 'libraries', 'book-thirdparty', 'seds-stripped', 'GMR_lib'), ...
        fullfile(projectRoot, 'libraries', 'book-thirdparty', 'seds-stripped')
    };
    for i = 1:numel(highPriority)
        p = highPriority{i};
        if isfolder(p)
            addpath(p, '-begin');
        end
    end

    % Add local project files without recursively adding every subfolder.
    % Recursive add of projectRoot can re-introduce conflicting toolboxes.
    addpath(projectRoot, '-begin');

    pathCandidates = {
        fullfile(projectRoot, 'hls'), ...
        fullfile(projectRoot, 'libraries', 'book-ds-opt'), ...
        fullfile(projectRoot, 'libraries', 'book-phys-gmm')
    };
    for i = 1:numel(pathCandidates)
        p = pathCandidates{i};
        if isfolder(p)
            addpath(genpath(p), '-begin');
        end
    end

    % Enforce stripped SEDS libs on top after all other path operations.
    for i = 1:numel(highPriority)
        p = highPriority{i};
        if isfolder(p)
            addpath(p, '-begin');
        end
    end

    % Final safety: remove any accidentally re-added SODS toolbox paths.
    sodsPath = fullfile(projectRoot, 'libraries', 'book-sods-opt');
    if isfolder(sodsPath)
        rmpath(genpath(sodsPath));
    end

    % Clear cached function definitions so new path precedence is applied.
    clear initialize_SEDS EM_init_kmeans EM_init_kmeans_SEDS SEDS_Solver;
    rehash;

    pathsInitialized = true;
end

function stateCallback(msg, bridge)
    data = double(msg.data(:));
    if numel(data) < 7
        return;
    end

    x = data(2:4);
    if any(~isfinite(x))
        return;
    end

    try
        xdot = bridge.ds.dsControl(x);
    catch err
        warning('[MATLAB DS Bridge] dsControl failed: %s', err.message);
        xdot = [0; 0; 0];
    end

    xdot = reshape(double(xdot), [3, 1]);
    if any(~isfinite(xdot))
        xdot = [0; 0; 0];
    end

    speed = norm(xdot);
    if speed > bridge.cfg.MaxLinearSpeed
        xdot = xdot / speed * bridge.cfg.MaxLinearSpeed;
    end

    cmdMsg = ros2message('std_msgs/Float64MultiArray');
    cmdMsg.data = xdot(:)';
    send(bridge.cmdPub, cmdMsg);
end

function shutdownBridge(bridge)
    fprintf('[MATLAB DS Bridge] Shutdown requested.\n');
    try
        cmdMsg = ros2message('std_msgs/Float64MultiArray');
        cmdMsg.data = [0, 0, 0];
        send(bridge.cmdPub, cmdMsg);
    catch
    end
end

function learnStatusCallback(msg, bridge)
    status = strtrim(char(msg.data));
    prefix = 'recording_stopped:';
    if ~startsWith(status, prefix)
        return;
    end

    datasetPath = strtrim(extractAfter(string(status), strlength(prefix)));
    datasetPath = char(datasetPath);

    if isempty(datasetPath) || strcmpi(datasetPath, 'none')
        warning('[MATLAB DS Bridge] recording_stopped received but no dataset path provided.');
        return;
    end

    if ~isfile(datasetPath)
        warning('[MATLAB DS Bridge] Dataset file not found: %s', datasetPath);
        return;
    end

    [~, ~, ext] = fileparts(datasetPath);
    if ~strcmpi(ext, '.mat')
        warning('[MATLAB DS Bridge] Expected .mat dataset for DS.m, got: %s', datasetPath);
        return;
    end

    try
        loaded = load(datasetPath);
        if ~isfield(loaded, 'trajectories')
            warning('[MATLAB DS Bridge] Variable "trajectories" not found in %s', datasetPath);
            return;
        end

        trajectories = loaded.trajectories;
        validateattributes(trajectories, {'numeric'}, {'nonempty', 'real', 'finite'});

        fprintf('[MATLAB DS Bridge] Learning SEDS from dataset: %s\n', datasetPath);
        bridge.ds.learnSEDS(trajectories);
        fprintf('[MATLAB DS Bridge] SEDS learning complete.\n');
    catch err
        warning('[MATLAB DS Bridge] Automatic SEDS learning failed: %s', err.message);
    end
end
