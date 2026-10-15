#include <gtest/gtest.h>
#include <amr_tim_localization/localizer.hpp>
#include <amr_tim_localization/port.hpp>
#include <relocalization/utils/relocalization_2d/sampling_2d.h>
#include <cmath>
#include <limits>
using namespace amr_tim_localization;
namespace {
nav_msgs::msg::OccupancyGrid map_fixture() {
 nav_msgs::msg::OccupancyGrid m; m.header.frame_id="map";
 m.info.width=120;m.info.height=90;m.info.resolution=0.1f;m.info.origin.orientation.w=1.0;
 m.data.assign(m.info.width*m.info.height,0);
 for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x) {
  if(x==0||y==0||x+1==m.info.width||y+1==m.info.height||(x>=75&&x<=80&&y<52)||(x>=20&&x<=37&&y>=65&&y<=71)||(x>=94&&x<=98&&y>=63&&y<=76)) m.data[y*m.info.width+x]=100;
 }
 return m;
}
sensor_msgs::msg::LaserScan observe(const nav_msgs::msg::OccupancyGrid &m,double x,double y,double yaw,bool reversed=false) {
 sensor_msgs::msg::LaserScan s;s.header.frame_id="laser";s.header.stamp.sec=123;s.header.stamp.nanosec=456;
 s.angle_min=reversed?M_PI:-M_PI;s.angle_increment=(reversed?-1:1)*2*M_PI/720.0;s.angle_max=s.angle_min+719*s.angle_increment;
 s.range_min=0.1;s.range_max=20.;s.ranges.resize(720);
 for(size_t i=0;i<s.ranges.size();++i) {
  const double a=yaw+s.angle_min+i*s.angle_increment;
  s.ranges[i]=std::numeric_limits<float>::infinity();
  for(double r=s.range_min;r<s.range_max;r+=.015) {
   const int xx=std::floor((x+r*std::cos(a))/m.info.resolution), yy=std::floor((y+r*std::sin(a))/m.info.resolution);
   if(xx<0||yy<0||xx>=static_cast<int>(m.info.width)||yy>=static_cast<int>(m.info.height))break;
   if(m.data[yy*m.info.width+xx]>=55){s.ranges[i]=r;break;}
  }
 }
 return s;
}
Service::Request request_fixture() {
 Service::Request r;r.map=map_fixture();r.scan=observe(r.map,3.4,4.7,1.1);r.timeout_sec=15.0;r.max_candidates=8;return r;
}
}
TEST(TimInput, RejectEmptyRequest) {
 Service::Request request;request.timeout_sec=1.0;request.max_candidates=3;Service::Response response;
 localize(request,response,{});EXPECT_FALSE(response.success);EXPECT_TRUE(response.poses.empty());EXPECT_NE(response.message.find("map"),std::string::npos);
}
TEST(TimInput, RejectInvalidScanAndMap) {
 auto r=request_fixture();Service::Response out;
 for(float &v:r.scan.ranges)v=std::numeric_limits<float>::quiet_NaN();
 localize(r,out,{});EXPECT_FALSE(out.success);EXPECT_NE(out.message.find("12 valid"),std::string::npos);
 r=request_fixture();r.map.info.origin.orientation.w=0;localize(r,out,{});EXPECT_FALSE(out.success);
 r=request_fixture();r.scan.angle_increment=0;localize(r,out,{});EXPECT_FALSE(out.success);
 r=request_fixture();r.map.data.pop_back();localize(r,out,{});EXPECT_FALSE(out.success);
}
TEST(TimAlgorithm, UnknownPoseWithReversedAngularIncrementAndRotatedMap) {
 auto r=request_fixture();r.scan=observe(r.map,3.4,4.7,1.1,true);
 constexpr double a=.61,ox=-3.2,oy=1.7;
 r.map.info.origin.position.x=ox;r.map.info.origin.position.y=oy;r.map.info.origin.orientation=tim_port::quaternion_yaw(a);
 Service::Response out;localize(r,out,{});ASSERT_TRUE(out.success)<<out.message;ASSERT_FALSE(out.poses.empty());
 const double gx=ox+std::cos(a)*3.4-std::sin(a)*4.7,gy=oy+std::sin(a)*3.4+std::cos(a)*4.7;
 bool found=false;
 for(const auto &p:out.poses) {
  EXPECT_EQ(p.header.frame_id,"map");EXPECT_EQ(p.header.stamp,r.scan.header.stamp);
  if(std::hypot(p.pose.position.x-gx,p.pose.position.y-gy)<.25 && std::abs(angles::shortest_angular_distance(a+1.1,tf2::getYaw(p.pose.orientation)))<.08)found=true;
 }
 EXPECT_TRUE(found)<<"cold start did not include correct unknown laser pose";
 for(double s:out.scores)EXPECT_TRUE(std::isfinite(s)&&s>=0&&s<=1);
}
TEST(TimSampling, OccupiedZeroAndDisconnectedRoomsBothCovered) {
 auto m=map_fixture();for(unsigned y=0;y<m.info.height;++y)m.data[y*m.info.width+60]=100;
 tim_port::deadline=tim_port::Clock::now()+std::chrono::seconds(10);
 const auto dist=calc_min_dist_to_obs_table(m,55);
 sampling_2d::sampling_2d sampling(.2f,2.f,.2f,.75f,true,30,55);sampling.initialize(m,dist);
 const auto points=sampling.get_points(0);ASSERT_FALSE(points.empty());bool left=false,right=false;
 for(const auto &p:points) {EXPECT_TRUE(is_point_in_map(p,m));EXPECT_LE(m.data[position_to_grid_index(p,m)],30);left|=p.x<5.9;right|=p.x>6.2;}
 EXPECT_TRUE(left);EXPECT_TRUE(right);
 for(size_t i=0;i<m.data.size();i+=17)if(m.data[i]==0&&dist[i]>=.2) {
  const auto p=grid_index_to_position(i,m);double closest=1e9;
  for(const auto &a:points)closest=std::min<double>(closest,calculate_distance(a,p));
  EXPECT_LE(closest,.751);
 }
}
TEST(TimDeadline, DiscardPartialResultsAndRemainUsable) {
 auto r=request_fixture();r.timeout_sec=.025;Service::Response out;
 localize(r,out,{});EXPECT_FALSE(out.success);EXPECT_TRUE(out.poses.empty());EXPECT_NE(out.message.find("deadline"),std::string::npos);EXPECT_LT(out.elapsed_sec,1.0);
 // A timed-out call cannot leak a stale candidate into the next invalid request.
 r.scan.ranges.assign(720,std::numeric_limits<float>::quiet_NaN());r.timeout_sec=1.0;
 localize(r,out,{});EXPECT_FALSE(out.success);EXPECT_TRUE(out.poses.empty());
}
TEST(TimScore, OffMapEndpointsCannotClampOntoBoundaryObstacles) {
 auto m=map_fixture();const auto table=calc_min_dist_to_obs_table(m,55);
 EXPECT_GT(calc_min_dist_to_obs(m,table,create_point(-.01f,4.f)),3.f);
 EXPECT_GT(calc_min_dist_to_obs(m,table,create_point(20.f,4.f)),3.f);
}
TEST(TimAlgorithm, KeepDistinctRivalsInDisconnectedIdenticalRooms) {
 auto r=request_fixture();const auto room=r.map;
 r.map.info.width=room.info.width*2;r.map.data.resize(r.map.info.width*r.map.info.height);
 for(unsigned y=0;y<room.info.height;++y)for(unsigned x=0;x<r.map.info.width;++x)
  r.map.data[y*r.map.info.width+x]=room.data[y*room.info.width+(x%room.info.width)];
 Service::Response out;localize(r,out,{});ASSERT_TRUE(out.success)<<out.message;
 bool first=false,second=false;
 for(const auto &p:out.poses) {
  const bool heading=std::abs(angles::shortest_angular_distance(1.1,tf2::getYaw(p.pose.orientation)))<.08;
  first|=heading&&std::hypot(p.pose.position.x-3.4,p.pose.position.y-4.7)<.25;
  second|=heading&&std::hypot(p.pose.position.x-15.4,p.pose.position.y-4.7)<.25;
 }
 EXPECT_TRUE(first);EXPECT_TRUE(second)<<"multi-candidate search must not hide a good disconnected rival";
}
TEST(TimAlgorithm, BackwardFacingScannerColdStart) {
 auto r=request_fixture();
 r.map.info.width=100;r.map.info.height=60;r.map.data.assign(6000,-1);
 for(int y=5;y<55;++y)for(int x=5;x<95;++x)r.map.data[y*100+x]=(x==5||x==94||y==5||y==54)?100:0;
 const int boxes[][4]={{35,55,20,29},{5,19,52,64},{31,40,68,80},{13,18,16,22}};
 for(const auto &b:boxes)for(int y=b[0];y<b[1];++y)for(int x=b[2];x<b[3];++x)r.map.data[y*100+x]=100;
 r.scan=observe(r.map,4.15,2.55,.43+M_PI);
 Service::Response out;localize(r,out,{});ASSERT_TRUE(out.success)<<out.message;
 bool found=false;for(const auto &p:out.poses) if(std::hypot(p.pose.position.x-4.15,p.pose.position.y-2.55)<.25&&std::abs(angles::shortest_angular_distance(.43+M_PI,tf2::getYaw(p.pose.orientation)))<.08)found=true;
 EXPECT_TRUE(found);
}
TEST(TimDeadline, TinyFieldOfViewCannotAllocateUnboundedVirtualScan) {
 auto r=request_fixture();r.scan.ranges.assign(12,1.0f);r.scan.angle_min=0;r.scan.angle_increment=1.1e-7f;r.scan.angle_max=11*r.scan.angle_increment;r.timeout_sec=.03;
 Service::Response out;localize(r,out,{});EXPECT_LT(out.elapsed_sec,1.0);
 EXPECT_FALSE(out.success);EXPECT_TRUE(out.poses.empty());
}
TEST(TimInput, RejectSkinnyMapBeforeDistanceTransformIntegerOverflow) {
 auto r=request_fixture();r.map.info.width=32769;r.map.info.height=2;r.map.data.assign(65538,0);
 Service::Response out;localize(r,out,{});EXPECT_FALSE(out.success);EXPECT_NE(out.message.find("dimensions"),std::string::npos);
}

#include <amr_tim_localization/fine_refinement.hpp>
TEST(TimFine, RefinesBiasedCandidateOnSameDenseMetric) {
 auto r=request_fixture();Config c;
 geometry_msgs::msg::TransformStamped seed;seed.header=r.scan.header;seed.header.frame_id="map";seed.child_frame_id="laser";
 seed.transform.translation.x=3.47;seed.transform.translation.y=4.76;seed.transform.rotation=tim_port::quaternion_yaw(1.11);
 tim_port::deadline=tim_port::Clock::now()+std::chrono::seconds(10);
 auto out=refine_candidates(r.map,r.scan,{seed},c,{.9f});
 ASSERT_EQ(out.poses.size(),1);ASSERT_EQ(out.diagnostics.size(),1);
 const auto &d=out.diagnostics[0];EXPECT_TRUE(d.accepted)<<d.reason<<" geometry="<<d.icp.geometry_ratio;
 EXPECT_GE(d.fine_score+1e-7,d.coarse_score);
 EXPECT_LT(std::hypot(out.poses[0].transform.translation.x-3.4,out.poses[0].transform.translation.y-4.7),.04);
 EXPECT_EQ(out.poses[0].header.stamp,seed.header.stamp);
}
TEST(TimFine, FineTimeBudgetPreservesAllCoarseCandidates) {
 auto r=request_fixture();Config c;c.fine_budget_sec=1e-9;
 geometry_msgs::msg::TransformStamped a;a.header=r.scan.header;a.transform.translation.x=3.47;a.transform.translation.y=4.76;a.transform.rotation=tim_port::quaternion_yaw(1.11);
 auto b=a;b.transform.translation.x=8.0;
 tim_port::deadline=tim_port::Clock::now()+std::chrono::seconds(10);
 const auto out=refine_candidates(r.map,r.scan,{a,b},c,{.9f,.8f});
 ASSERT_EQ(out.poses.size(),2);EXPECT_EQ(out.poses[0],a);EXPECT_EQ(out.poses[1],b);
 for(const auto &d:out.diagnostics){EXPECT_FALSE(d.accepted);EXPECT_EQ(d.reason,"fine_deadline");}
 EXPECT_FALSE(tim_port::expired());
}
TEST(TimFine, RejectsIterationCapRegressionLargeMotionAndDegeneracy) {
 Config c;FineDiagnostic d;d.coarse_score=.9;d.fine_score=.95;d.icp.converged=true;d.icp.termination=2;d.icp.rms=.02;d.icp.overlap=.95;d.icp.geometry_ratio=.1;d.icp.normal_constraints=100;
 EXPECT_TRUE(fine_rejection(d,c,false).empty());
 d.icp.termination=1;EXPECT_EQ(fine_rejection(d,c,false),"icp_not_converged");d.icp.termination=2;
 d.fine_score=.89;EXPECT_EQ(fine_rejection(d,c,false),"insufficient_dense_gain");d.fine_score=.95;
 d.translation=.3;EXPECT_EQ(fine_rejection(d,c,false),"excessive_correction");d.translation=0;
 d.icp.geometry_ratio=0;EXPECT_EQ(fine_rejection(d,c,false),"degenerate_geometry");d.icp.geometry_ratio=.1;
 d.icp.overlap=.2;EXPECT_EQ(fine_rejection(d,c,false),"low_overlap");
 EXPECT_EQ(fine_rejection(d,c,true),"fine_deadline");
}
TEST(TimFine, ParallelWallsExposeUnobservableDirection) {
 auto r=request_fixture();r.map.info.width=300;r.map.info.height=60;r.map.data.assign(18000,0);
 for(int x=0;x<300;++x){r.map.data[10*300+x]=100;r.map.data[50*300+x]=100;}
 r.scan=observe(r.map,15.0,3.0,.4);r.scan.range_max=6.;Config c;c.fine_correspondence_distance=.2;
 geometry_msgs::msg::TransformStamped seed;seed.header=r.scan.header;seed.transform.translation.x=15.01;seed.transform.translation.y=3.03;seed.transform.rotation=tim_port::quaternion_yaw(.4);
 tim_port::deadline=tim_port::Clock::now()+std::chrono::seconds(10);
 auto out=refine_candidates(r.map,r.scan,{seed},c,{.9f});ASSERT_EQ(out.poses.size(),1);
 EXPECT_FALSE(out.diagnostics[0].accepted);EXPECT_EQ(out.diagnostics[0].reason,"degenerate_geometry");EXPECT_EQ(out.poses[0],seed);
}
TEST(TimFine, InsufficientDenseSamplesRestoreOriginalMetricForAllCandidates) {
 auto r=request_fixture();Config c;c.fine_max_beams=12;
 for(size_t i=0;i<r.scan.ranges.size();i+=60)r.scan.ranges[i]=std::numeric_limits<float>::quiet_NaN();
 geometry_msgs::msg::TransformStamped a;a.transform.translation.x=3.4;a.transform.translation.y=4.7;a.transform.rotation=tim_port::quaternion_yaw(1.1);
 auto b=a;b.transform.translation.x=8.;
 tim_port::deadline=tim_port::Clock::now()+std::chrono::seconds(10);
 const auto out=refine_candidates(r.map,r.scan,{a,b},c,{.91f,.86f});
 EXPECT_EQ(out.poses,(std::vector<geometry_msgs::msg::TransformStamped>{a,b}));
 EXPECT_EQ(out.scores,(std::vector<float>{.91f,.86f}));
 for(const auto &d:out.diagnostics)EXPECT_EQ(d.reason,"insufficient_dense_returns");
}
TEST(TimFine, EntireRequestDeadlineStillFailsClosed) {
 auto r=request_fixture();Config c;
 geometry_msgs::msg::TransformStamped seed;seed.transform.translation.x=3.4;seed.transform.translation.y=4.7;seed.transform.rotation=tim_port::quaternion_yaw(1.1);
 tim_port::deadline=tim_port::Clock::now()-std::chrono::seconds(1);
 EXPECT_THROW(refine_candidates(r.map,r.scan,{seed},c,{.9f}),std::runtime_error);
 EXPECT_TRUE(tim_port::expired());
}
TEST(TimFine, ScoreTiesCannotMoveCandidate) {
 Config c;FineDiagnostic d;d.coarse_score=d.fine_score=.95;d.icp.converged=true;d.icp.termination=2;d.icp.rms=.01;d.icp.overlap=.99;d.icp.geometry_ratio=.1;d.icp.normal_constraints=100;
 EXPECT_EQ(fine_rejection(d,c,false),"insufficient_dense_gain");
 d.fine_score=std::numeric_limits<double>::quiet_NaN();EXPECT_EQ(fine_rejection(d,c,false),"nonfinite");
}
