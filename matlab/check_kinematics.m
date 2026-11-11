% check_kinematics.m - independent MATLAB check of the C++ kinmp FK + Jacobians.
% Rebuilds both DH chains (Panda modified DH, palletizer standard DH with the coupled
% level-keeping wrist) from scratch, recomputes FK and the geometric Jacobian for every row of
% fk_samples.csv (exported by the C++ GoogleTest test), plus a central finite-difference
% Jacobian, and writes the max errors to matlab_check.txt.
% Runs in MATLAB (R2016b+) and GNU Octave. No toolboxes needed.
% Usage (MATLAB Online): upload this file + fk_samples.csv to the same folder, run:
%   check_kinematics
if ~exist('csv_file', 'var'), csv_file = 'fk_samples.csv'; end
if ~exist('out_file', 'var'), out_file = 'matlab_check.txt'; end

D = dlmread(csv_file, ',', 1, 0);          % skip header row
Rz = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];
Rx = @(t) [1 0 0 0; 0 cos(t) -sin(t) 0; 0 sin(t) cos(t) 0; 0 0 0 1];
Tr = @(x, y, z) [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
h = pi / 2;
% chain 1: Panda, modified DH rows [a alpha d], tool = Tz(0.107) Rz(3pi/4)
P.dh = [0 0 0.333; 0 -h 0; 0 h 0.316; 0.0825 h 0; -0.0825 -h 0.384; 0 h 0; 0.088 h 0];
P.modified = true; P.C = eye(7); P.tool = Tr(0, 0, 0.107) * Rz(3 * pi / 4); P.n = 7;
% chain 2: palletizer, standard DH rows [a alpha d], q_full = C * q
L.dh = [0 h 0.5; 1.0 0 0; 0.9 0 0; 0.2 h 0; 0 0 0.25];
L.modified = false; L.C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 -1 -1 0; 0 0 0 1]; L.tool = eye(4); L.n = 4;
chains = {P, L};
names = {'panda', 'palletizer'};

fid = fopen(out_file, 'w');
if exist('OCTAVE_VERSION', 'builtin'), eng = ['GNU Octave ' OCTAVE_VERSION]; else, eng = ['MATLAB ' version]; end
fprintf(fid, 'engine: %s\ncsv: %s (%d rows)\n', eng, csv_file, size(D, 1));
for c = 1:2
  K = chains{c};
  rows = find(D(:, 1) == c);
  max_p = 0; max_R = 0; max_J = 0; max_Jfd = 0;
  for s = rows'
    q = D(s, 2:1 + K.n)';
    p_cpp = D(s, 9:11)';
    R_cpp = reshape(D(s, 12:20), 3, 3)';           % row-major in the CSV
    J_cpp = reshape(D(s, 21:62), 6, 7); J_cpp = J_cpp(:, 1:K.n);  % column-major 6x7
    % --- FK with all frames ---
    qf = K.C * q; nj = size(K.dh, 1);
    T = eye(4); F = cell(nj + 1, 1); F{1} = T;
    for i = 1:nj
      a = K.dh(i, 1); al = K.dh(i, 2); d = K.dh(i, 3);
      if K.modified
        A = Rx(al) * Tr(a, 0, 0) * Rz(qf(i)) * Tr(0, 0, d);
      else
        A = Rz(qf(i)) * Tr(0, 0, d) * Tr(a, 0, 0) * Rx(al);
      end
      T = T * A; F{i + 1} = T;
    end
    Te = T * K.tool; pe = Te(1:3, 4);
    % --- geometric Jacobian ---
    Jf = zeros(6, nj);
    for i = 1:nj
      if K.modified, Fi = F{i + 1}; else, Fi = F{i}; end
      z = Fi(1:3, 3); o = Fi(1:3, 4);
      Jf(:, i) = [cross(z, pe - o); z];
    end
    J = Jf * K.C;
    max_p = max(max_p, norm(pe - p_cpp));
    max_R = max(max_R, max(max(abs(Te(1:3, 1:3) - R_cpp))));
    max_J = max(max_J, max(max(abs(J - J_cpp))));
    % --- central finite-difference Jacobian of the MATLAB FK ---
    hh = 1e-6; Jfd = zeros(6, K.n);
    for k = 1:K.n
      dq = zeros(K.n, 1); dq(k) = hh;
      TT = cell(2, 1); sg = [1 -1];
      for w = 1:2                      % FK at q + dq and q - dq (inline: Octave-safe script)
        qq = K.C * (q + sg(w) * dq); Tw = eye(4);
        for i = 1:nj
          a = K.dh(i, 1); al = K.dh(i, 2); d = K.dh(i, 3);
          if K.modified, A = Rx(al) * Tr(a, 0, 0) * Rz(qq(i)) * Tr(0, 0, d);
          else, A = Rz(qq(i)) * Tr(0, 0, d) * Tr(a, 0, 0) * Rx(al); end
          Tw = Tw * A;
        end
        TT{w} = Tw * K.tool;
      end
      Tp = TT{1}; Tm = TT{2};
      Jfd(1:3, k) = (Tp(1:3, 4) - Tm(1:3, 4)) / (2 * hh);
      W = (Tp(1:3, 1:3) * Tm(1:3, 1:3)' - Tm(1:3, 1:3) * Tp(1:3, 1:3)') / 2;  % skew part ~ [w]x * 2h
      Jfd(4:6, k) = [W(3, 2); W(1, 3); W(2, 1)] / (2 * hh);
    end
    max_Jfd = max(max_Jfd, max(max(abs(J - Jfd))));
  end
  fprintf(fid, '%s: samples=%d fk_pos_max_err_m=%.3e fk_R_max_abs_err=%.3e jacobian_max_abs_err_vs_cpp=%.3e jacobian_max_abs_err_vs_matlab_fd=%.3e\n', ...
          names{c}, numel(rows), max_p, max_R, max_J, max_Jfd);
end
fclose(fid);
type(out_file);

