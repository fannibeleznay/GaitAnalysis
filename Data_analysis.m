%% Data Analysis

load('measure_70_20.mat')
load('measure_70_0.mat')
load('measure_60_20.mat')
load('measure_60_0.mat')
load('measure_50_20.mat')
load('measure_50_0.mat')
load('measure_40_20.mat')
load('measure_40_0.mat')
load('measure_30_20.mat')
load('measure_30_0.mat')
load('measure_20_20.mat')
load('measure_20_0.mat')

figure(1);
boxplot([measure_20_20(:,1),measure_30_20(:,1),measure_40_20(:,1),measure_50_20(:,1),measure_60_20(:,1),measure_70_20(:,1)],'Orientation','horizontal')
title('Distances at 20 deg')

figure(2);
boxplot([measure_20_20(:,2),measure_30_20(:,2),measure_40_20(:,2),measure_50_20(:,2),measure_60_20(:,2),measure_70_20(:,2)])
title('20 deg at increasing distances')

figure(3);
boxplot([measure_20_0(:,1),measure_30_0(:,1),measure_40_0(:,1),measure_50_0(:,1),measure_60_0(:,1),measure_70_0(:,1)],'Orientation','horizontal')
title('Distances at 0 deg')

figure(4);
boxplot([measure_20_0(:,2),measure_30_0(:,2),measure_40_0(:,2),measure_50_0(:,2),measure_60_0(:,2),measure_70_0(:,2)])
title('0 deg at increasing distance')