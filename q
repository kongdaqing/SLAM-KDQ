[1mdiff --git a/SlamCodes/viz_scene/src/camera_project/camera_project.cpp b/SlamCodes/viz_scene/src/camera_project/camera_project.cpp[m
[1mindex 1bd1123..4be4920 100644[m
[1m--- a/SlamCodes/viz_scene/src/camera_project/camera_project.cpp[m
[1m+++ b/SlamCodes/viz_scene/src/camera_project/camera_project.cpp[m
[36m@@ -25,7 +25,7 @@[m [mProjectPointInfo CameraProject::projectVizPoints(double t,const std::vector<cv::[m
     return ProjectPointInfo();[m
   }[m
   std::mt19937 gen{12345};[m
[31m-  std::normal_distribution<> d{0.0, pixelNoise_};[m
[32m+[m[32m  std::normal_distribution<float> d{0.0, pixelNoise_};[m
 [m
   Eigen::Isometry3d Tcw = Twc.inverse();[m
   ProjectPointInfo ptsInfo;[m
