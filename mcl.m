function out = mcl(bagFile)

    if nargin < 1 || isempty(bagFile)
        [f,p] = uigetfile('*.bag','Selecciona un rosbag');
        if isequal(f,0)
            error('No se selecciono rosbag.');
        end
        bagFile = fullfile(p,f);
    end

    bag = rosbag(bagFile);
    topics = bag.AvailableTopics.Properties.RowNames;
    hasTopic = @(name) any(strcmp(name, topics));

    
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));
    wrapPi = @(a) atan2(sin(a), cos(a));

    if ~hasTopic('/map')
        error('Falta /map');
    end
    bagMap = select(bag,'Topic','/map');
    mapMsg = readMessages(bagMap,1,'DataFormat','struct');
    mapMsg = mapMsg{1};

    res = double(mapMsg.Info.Resolution);
    W = double(mapMsg.Info.Width);
    H = double(mapMsg.Info.Height);

    origin_x = double(mapMsg.Info.Origin.Position.X);
    origin_y = double(mapMsg.Info.Origin.Position.Y);
    origin_yaw = quatToYaw(mapMsg.Info.Origin.Orientation);

    dataRaw = int16(mapMsg.Data(:));
    occgrid = reshape(dataRaw, [W, H])';
    occThresh = 50;
    occ = (occgrid >= occThresh);

    distCells = bwdist(occ);
    dist = double(distCells) * res;

    laser_offset = [0;0];
    laser_yaw = -3*pi/4;

    if hasTopic('/tf_static')
        bagTFS = select(bag,'Topic','/tf_static');
        tfsMsgs = readMessages(bagTFS,1,'DataFormat','struct');
        tfs = tfsMsgs{1}.Transforms;
        found = false;
        for i = 1:numel(tfs)
            ch = string(tfs(i).ChildFrameId);
            pr = string(tfs(i).Header.FrameId);
            if ch == "back_laser_link" && (pr == "base_link" || pr == "base_footprint")
                tr = tfs(i).Transform.Translation;
                rq = tfs(i).Transform.Rotation;
                laser_offset = [double(tr.X); double(tr.Y)];
                laser_yaw = quatToYaw(struct('W',rq.W,'X',rq.X,'Y',rq.Y,'Z',rq.Z));
                found = true;
                break;
            end
        end
        if ~found
            for i = 1:numel(tfs)
                ch = string(tfs(i).ChildFrameId);
                pr = string(tfs(i).Header.FrameId);
                if ch == "back_laser_link" && pr == "base_link"
                    tr = tfs(i).Transform.Translation;
                    rq = tfs(i).Transform.Rotation;
                    laser_offset = [double(tr.X); double(tr.Y)];
                    laser_yaw = quatToYaw(struct('W',rq.W,'X',rq.X,'Y',rq.Y,'Z',rq.Z));
                    break;
                end
            end
        end
    end

    odomTopic = '/odometry/filtered';
    if ~hasTopic(odomTopic)
        error('Falta %s', odomTopic);
    end

    bagOdom = select(bag,'Topic',odomTopic);
    odomMsgs = readMessages(bagOdom,'DataFormat','struct');

    nO = numel(odomMsgs);
    odom.t   = zeros(nO,1);
    odom.x   = zeros(nO,1);
    odom.y   = zeros(nO,1);
    odom.yaw = zeros(nO,1);

    for k = 1:nO
        m = odomMsgs{k};
        odom.t(k) = timeFromHeader(m.Header);
        odom.x(k) = m.Pose.Pose.Position.X;
        odom.y(k) = m.Pose.Pose.Position.Y;
        q = m.Pose.Pose.Orientation;
        odom.yaw(k) = quatToYaw(q);
    end

    [odom.t, idxO] = sort(odom.t);
    odom.x   = odom.x(idxO);
    odom.y   = odom.y(idxO);
    odom.yaw = odom.yaw(idxO);

    odom.t0 = odom.t(1);
    odom.tr = odom.t - odom.t0;

    inc.dt   = zeros(nO,1);
    inc.dx   = zeros(nO,1);
    inc.dy   = zeros(nO,1);
    inc.dyaw = zeros(nO,1);

    inc.dt(1) = NaN;
    for k = 2:nO
        inc.dt(k) = odom.tr(k) - odom.tr(k-1);
        inc.dx(k) = odom.x(k) - odom.x(k-1);
        inc.dy(k) = odom.y(k) - odom.y(k-1);
        inc.dyaw(k) = wrapPi(odom.yaw(k) - odom.yaw(k-1));
    end
    odom.inc = inc;

    scanTopic = '/scan';
    if ~hasTopic(scanTopic)
        error('Falta %s', scanTopic);
    end

    bagScan = select(bag,'Topic',scanTopic);
    scanMsgs = readMessages(bagScan,'DataFormat','struct');

    nS = numel(scanMsgs);
    scan.t  = zeros(nS,1);
    scan.ranges = cell(nS,1);

    s0 = scanMsgs{1};
    scan.frame_id  = s0.Header.FrameId;
    scan.angle_min = double(s0.AngleMin);
    scan.angle_inc = double(s0.AngleIncrement);
    scan.range_min = double(s0.RangeMin);
    scan.range_max = double(s0.RangeMax);
    scan.N = numel(s0.Ranges);
    scan.angles = scan.angle_min + (0:scan.N-1)' * scan.angle_inc;

    for i = 1:nS
        s = scanMsgs{i};
        scan.t(i) = timeFromHeader(s.Header);
        scan.ranges{i} = double(s.Ranges(:));
    end

    [scan.t, idxS] = sort(scan.t);
    scan.ranges = scan.ranges(idxS);
    scan.tr = scan.t - odom.t0;

    maxScanAge = 0.08;
    assoc.scan_idx = zeros(nO,1);
    assoc.scan_dt  = NaN(nO,1);
    assoc.maxScanAge = maxScanAge;

    j = 1;
    for k = 1:nO
        tk = odom.tr(k);
        while j < nS && scan.tr(j) < tk
            j = j + 1;
        end

        cand = [];
        if j >= 1 && j <= nS, cand(end+1) = j; end
        if (j-1) >= 1 && (j-1) <= nS, cand(end+1) = j-1; end

        if isempty(cand)
            assoc.scan_idx(k) = 0;
            assoc.scan_dt(k)  = NaN;
            continue;
        end

        [dtmin, ii] = min(abs(scan.tr(cand) - tk));
        best = cand(ii);

        if dtmin <= maxScanAge
            assoc.scan_idx(k) = best;
            assoc.scan_dt(k)  = dtmin;
        else
            assoc.scan_idx(k) = 0;
            assoc.scan_dt(k)  = dtmin;
        end
    end

    Np = 600;
    sigma_init_xy  = 0.30;
    sigma_init_yaw = deg2rad(20);

    alpha_xy   = 0.02;
    alpha_yaw  = deg2rad(0.5);
    alpha_xy_r = 0.01;
    alpha_yaw_t = deg2rad(1.0);


    sigma_z = 0.15;
    maxDistFallback = 1.0;
    beamStep = 8;
    beamIdxAll = 1:beamStep:scan.N;

    NeffThreshFrac = 0.5;

    particles = zeros(Np,3);
    weights   = ones(Np,1) / Np;

    x0 = odom.x(1);
    y0 = odom.y(1);
    yaw0 = odom.yaw(1);
    particles(:,1) = x0 + sigma_init_xy  * randn(Np,1);
    particles(:,2) = y0 + sigma_init_xy  * randn(Np,1);
    particles(:,3) = wrapPi(yaw0 + sigma_init_yaw * randn(Np,1));
    est = zeros(nO,3);
    Neff = zeros(nO,1);
    didResample = false(nO,1);

    est(1,:) = [sum(particles(:,1).*weights), sum(particles(:,2).*weights), circMeanAngle(particles(:,3), weights)];
    Neff(1) = 1/sum(weights.^2);

    for k = 2:nO

        dx = odom.inc.dx(k);
        dy = odom.inc.dy(k);
        dyaw = odom.inc.dyaw(k);

        trans = hypot(dx, dy);
        rot = abs(dyaw);

        sig_xy = alpha_xy + alpha_xy_r * trans;
        sig_yaw = alpha_yaw + alpha_yaw_t * rot;

        ndx = dx + sig_xy * randn(Np,1);
        ndy = dy + sig_xy * randn(Np,1);
        ndyaw = dyaw + sig_yaw * randn(Np,1);
        particles(:,1) = particles(:,1) + ndx;
        particles(:,2) = particles(:,2) + ndy;
        %{
        particles(:,1) = particles(:,1) + ndx .* cos(particles(:,3)) - ndy .* sin(particles(:,3));
        particles(:,2) = particles(:,2) + ndx .* sin(particles(:,3)) + ndy
        .* cos(particles(:,3));
        %}
        particles(:,3) = wrapPi(particles(:,3) + ndyaw);

        idxScan = assoc.scan_idx(k);

        if idxScan > 0
            ranges = scan.ranges{idxScan};
            angles = scan.angles;

            bi = beamIdxAll;
            r = ranges(bi);
            a = angles(bi);

            valid = isfinite(r) & (r > (scan.range_min + 1e-3)) & (r < (scan.range_max - 1e-3));
            r = r(valid);
            a = a(valid);

            if ~isempty(r)
                logw = zeros(Np,1);

                ca = cos(a);
                sa = sin(a);

                lx = laser_offset(1);
                ly = laser_offset(2);

                for p = 1:Np
                    xp = particles(p,1);
                    yp = particles(p,2);
                    th = particles(p,3);

                    cth = cos(th);
                    sth = sin(th);

                    lpx = xp + cth*lx - sth*ly;
                    lpy = yp + sth*lx + cth*ly;
                    lth = th + laser_yaw;

                    cl = cos(lth);
                    sl = sin(lth);

                    ex = lpx + r .* (cl .* ca - sl .* sa);
                    ey = lpy + r .* (sl .* ca + cl .* sa);

                    mx = (ex - origin_x) / res;
                    my = (ey - origin_y) / res;

                    ix = floor(mx) + 1;
                    iy = floor(my) + 1;

                    in = (ix >= 1) & (ix <= W) & (iy >= 1) & (iy <= H);

                    d = maxDistFallback * ones(size(ix));
                    if any(in)
                        lin = sub2ind([H, W], iy(in), ix(in));
                        d(in) = dist(lin);
                    end

                    logw(p) = -0.5 * sum((d.^2) / (sigma_z^2));
                end

                mlog = max(logw);
                wnew = exp(logw - mlog);
                sW = sum(wnew);
                if sW <= 0 || ~isfinite(sW)
                    wnew = ones(Np,1)/Np;
                else
                    wnew = wnew / sW;
                end
                weights = wnew;

                Neff(k) = 1/sum(weights.^2);
               
                if Neff(k) < (NeffThreshFrac * Np)
                    idxR = systematicResample(weights);
                    particles = particles(idxR,:);
                    weights = ones(Np,1)/Np;
                    didResample(k) = true;
                    Neff(k) = Np;
                end
            else
                Neff(k) = 1/sum(weights.^2);
            end
        else
            Neff(k) = 1/sum(weights.^2);
        end

        est(k,:) = [sum(particles(:,1).*weights), sum(particles(:,2).*weights), circMeanAngle(particles(:,3), weights)];
    end

    out = struct();
    out.bagFile = bagFile;
    out.map = struct('res',res,'W',W,'H',H,'origin_x',origin_x,'origin_y',origin_y,'origin_yaw',origin_yaw,'occ',occ);
    out.dist = dist;
    out.laser = struct('offset',laser_offset,'yaw',laser_yaw,'frame_id',scan.frame_id);
    out.odom = odom;
    out.scan = scan;
    out.assoc = assoc;
    out.est = est;
    out.Neff = Neff;
    out.didResample = didResample;
    out.particles_end = particles;
    out.weights_end = weights;

    if hasTopic('/base_pose_ground_truth')
        bagGT = select(bag,'Topic','/base_pose_ground_truth');
        gtMsgs = readMessages(bagGT,'DataFormat','struct');
        nG = numel(gtMsgs);

        gt.t   = zeros(nG,1);
        gt.x   = zeros(nG,1);
        gt.y   = zeros(nG,1);
        gt.yaw = zeros(nG,1);

        for k = 1:nG
            m = gtMsgs{k};
            gt.t(k) = timeFromHeader(m.Header);
            gt.x(k) = m.Pose.Pose.Position.X;
            gt.y(k) = m.Pose.Pose.Position.Y;
            q = m.Pose.Pose.Orientation;
            gt.yaw(k) = quatToYaw(q);
        end

        [gt.t, idxG] = sort(gt.t);
        gt.x = gt.x(idxG); gt.y = gt.y(idxG); gt.yaw = gt.yaw(idxG);
        gt.tr = gt.t - odom.t0;
        out.gt = gt;

        gx = interp1(gt.tr, gt.x, odom.tr, 'linear', 'extrap');
        gy = interp1(gt.tr, gt.y, odom.tr, 'linear', 'extrap');
        gyaw = interp1(gt.tr, unwrap(gt.yaw), odom.tr, 'linear', 'extrap');
        gyaw = wrapPi(gyaw);

        ex = est(:,1) - gx;
        ey = est(:,2) - gy;
        epos = hypot(ex, ey);
        eyaw = wrapPi(est(:,3) - gyaw);

        out.err = struct('epos',epos,'eyaw',eyaw,'gx',gx,'gy',gy,'gyaw',gyaw);
    end

    figure('Name','Paso 4: Trayectorias','NumberTitle','off');
    hold on; grid on; axis equal;
    plot(odom.x, odom.y, 'b--', 'DisplayName','Odom filtered');
    plot(est(:,1), est(:,2), 'r-', 'DisplayName','MCL (predict+update)');
    if isfield(out,'gt')
        plot(out.gt.x, out.gt.y, 'k-', 'DisplayName','Ground truth');
    end
    legend('Location','best');
    xlabel('x [m]'); ylabel('y [m]');
    title('MCL con update (scan+map)');
    hold off;

    figure('Name','Paso 4: N_eff y resampling','NumberTitle','off');
    hold on; grid on;
    plot(odom.tr, Neff, 'DisplayName','N_{eff}');
    yline(NeffThreshFrac*Np, '--', 'DisplayName','Umbral');
    stem(odom.tr(didResample), Neff(didResample), '.', 'DisplayName','Resample');
    legend('Location','best');
    xlabel('t [s]'); ylabel('N_{eff}');
    title('Degeneracion y resampling');
    hold off;

    if isfield(out,'err')
        figure('Name','Paso 4: Error vs tiempo','NumberTitle','off');
        tiledlayout(2,1);
        nexttile;
        plot(odom.tr, out.err.epos);
        grid on;
        xlabel('t [s]'); ylabel('error pos [m]');
        title('Error de posicion vs GT');
        nexttile;
        plot(odom.tr, out.err.eyaw);
        grid on;
        xlabel('t [s]'); ylabel('error yaw [rad]');
        title('Error de yaw vs GT');
    end

    fprintf('Paso 4 listo: update con laser+map, Neff y resampling.\n');

    kTest = find(assoc.scan_idx > 0, 1, 'first') + 200;
    showScanOnMap(kTest, 0);
    showScanOnMap(kTest, 1);

    function ang = circMeanAngle(a,w)
        s = sum(sin(a).*w);
        c = sum(cos(a).*w);
        ang = atan2(s,c);
    end

    function showScanOnMap(kOdom, pIdx)
        idxScan = assoc.scan_idx(kOdom);
        if idxScan <= 0
            error('No hay scan asociado en k=%d', kOdom);
        end

        ranges = scan.ranges{idxScan};
        angles = scan.angles;

        beamStepVis = 1;
        bi = 1:beamStepVis:numel(ranges);

        r = ranges(bi);
        a = angles(bi);
        valid = isfinite(r) & (r > scan.range_min) & (r < scan.range_max);
        r = r(valid);
        a = a(valid);

        if isempty(r)
            error('Scan sin rayos validos en k=%d', kOdom);
        end

        if pIdx == 0

            xp = interp1(out.gt.tr, out.gt.x, odom.tr(kOdom), 'linear', 'extrap');
            yp = interp1(out.gt.tr, out.gt.y, odom.tr(kOdom), 'linear', 'extrap');
            th = interp1(out.gt.tr, unwrap(out.gt.yaw), odom.tr(kOdom), 'linear', 'extrap');
            th = wrapPi(th);
            tag = ''
            xp = est(kOdom,1);
            yp = est(kOdom,2);
            th = est(kOdom,3);
            tag = 'EST';

        else
            xp = particles(pIdx,1);
            yp = particles(pIdx,2);
            th = particles(pIdx,3);

            tag = sprintf('P%04d', pIdx);
        end

        lx = laser_offset(1);
        ly = laser_offset(2);

        cth = cos(th);
        sth = sin(th);

        lpx = xp + cth*lx - sth*ly;
        lpy = yp + sth*lx + cth*ly;
        lth = th + laser_yaw;

        ca = cos(a);
        sa = sin(a);

        cl = cos(lth);
        sl = sin(lth);

        ex = lpx + r .* (cl .* ca - sl .* sa);
        ey = lpy + r .* (sl .* ca + cl .* sa);

        figure('Name',sprintf('Scan sobre mapa | k=%d',kOdom),'NumberTitle','off');

        imagesc([origin_x, origin_x + W*res], [origin_y, origin_y + H*res], occ);
        set(gca,'YDir','normal')
        colormap(gray);
        axis equal; axis tight; hold on;

        plot(xp, yp, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        quiver(xp, yp, cos(th), sin(th), 0.6, 'r', 'LineWidth', 2);

        plot(lpx, lpy, 'mo', 'MarkerSize', 6, 'LineWidth', 2);

        scatter(ex, ey, 10, 'g', 'filled');

        M = min(30, numel(ex));
        sel = round(linspace(1, numel(ex), M));
        for ii = sel
            plot([lpx ex(ii)], [lpy ey(ii)], 'g-');
        end

        title(sprintf('Scan proyectado en mapa (verde) | robot=%s', tag));
        xlabel('x [m]'); ylabel('y [m]');
        hold off;
    end


    function idx = systematicResample(w)
        N = numel(w);
        edges = cumsum(w);
        edges(end) = 1.0;
        u0 = rand()/N;
        u = u0 + (0:N-1)'/N;
        idx = zeros(N,1);
        i = 1;
        for j = 1:N
            while u(j) > edges(i)
                i = i + 1;
            end
            idx(j) = i;
        end
    end

end
