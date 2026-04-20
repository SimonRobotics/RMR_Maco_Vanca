#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <QQueue>
#include <QDebug>


#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)

#define TICK_TO_METER 0.000085292090497737556558
#define MAX_SPEED 200 //mm / s
#define MAX_SPEED_ANG 0.4 //stupen / s
#define MAP_SIZE_METERS 14//m
#define PIXEL_PER_METER 20// px / m
#define ROBOT_SIZE_MM 350 /mm

struct Position{
    double x;
    double y;
};

struct TimePosition{
    double timeStamp;
    double angle;
    Position pos;
};

struct Point{
    int x;
    int y;
};


class robot : public QObject {
  Q_OBJECT
public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);
  void setState(int state);
  int getState();
  void addWaypoint(double x, double y);
  std::vector<Point> getMap();
  std::vector<Point> getCostMap();

signals:
  void publishPosition(double x, double y, double z);
  void publishMap(std::vector<Point> pointList);
  void publishWaypoints(QQueue<Position> waypointList);
  void publishLidar(const std::vector<LaserData> &lidata);
  void resetMap();

#ifndef DISABLE_OPENCV
  void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
  void publishSkeleton(const skeleton &skeledata);
#endif
private:
  /// toto su vase premenne na vasu odometriu

  bool initParam;
  bool newLidarData;
  bool createCostmap;
  double d;
  double gyroOffSet;
  int state;
  int map[MAP_SIZE_METERS*PIXEL_PER_METER][MAP_SIZE_METERS*PIXEL_PER_METER];

  unsigned short lastValueLeft;
  unsigned short lastValueRight;


  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  int processThisRobot(const TKobukiData &robotdata);
  double ramp(double target, double d, double y);
  double calculateDistanceError(Position setPoint, double x, double y);
  double calculateAngleError(Position setPoint, double x, double y, double fi);
  double getDistanceFromWhells(double leftWheel, double rightWheel);
  double realDistanceTraveled(unsigned short encoderValue, unsigned short *LastValue);
  double distance_polar(double r1, double theta1, double r2, double theta2);
  double interpolate(double x1, double x2, double t1, double t2, double t);
  double interpolateAngle(double a1, double a2, double t1, double t2, double t);
  void printToMap(Position pos);
  void createCostMap(int numOfPixels);
  int createPath(Point p);
  QQueue<Position> getPathKeyPositions();
  std::vector<Point> findElementAroundPointCross(Point p, int element);
  std::vector<Point> findElementAroundPoint(Point p, int element);
  std::vector<Point> findLowerThenElementAroundPoint(Point p, int element);
  int findDirection(Point last, Point point);
  int sign(double x);
  Point xyToMapTransform(Position pos);
  Position mapToXYTransform(Point p);
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  QQueue<Position> position_list;

  std::vector<TimePosition> pastPositions;

  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;
};

#endif // ROBOT_H
