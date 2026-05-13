    #include "robot.h"

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{
    useDirectCommands = 0;
    initParam = false;
    newLidarData = false;
    createCostmap = true;
    navigation = true;
    d = 10; // s
    forwardspeed=0;
    rotationspeed=0;
    state = 0;

    binHistogram.assign(36,0);

    selectedDirection = 0.0;
    obstacleAvoidanceActive = false;
    frontBlocked = false;
    goalDirection = 0.0;
    previousSelectedDirection = 0.0;
    mclCounter = 0;


    for (int i = 0; i< MAP_SIZE_METERS*PIXEL_PER_METER;i++){
        for (int j = 0; j< MAP_SIZE_METERS*PIXEL_PER_METER;j++){
            map[i][j] = 0;
        }
    }

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress); 

  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::addWaypoint(double x, double y)
{
    Position pos;
    pos.x = x;
    pos.y = y;
    position_list.push_back(pos);
}

std::vector<Point> robot::getMap()
{
    std::vector<Point> mm;
    for (int i = 0; i< MAP_SIZE_METERS*PIXEL_PER_METER;i++){
        for (int j = 0; j< MAP_SIZE_METERS*PIXEL_PER_METER;j++){
            if (map[i][j] == 1|| map[i][j] == 7 || map[i][j] == 9){
                Point p;
                p.x = i;
                p.y = j;
                mm.push_back(p);
            }
        }
    }
    return mm;
}

std::vector<Point> robot::getCostMap()
{
    std::vector<Point> mm;
    for (int i = 0; i< MAP_SIZE_METERS*PIXEL_PER_METER;i++){
        for (int j = 0; j< MAP_SIZE_METERS*PIXEL_PER_METER;j++){
            if (map[i][j] > 1){
                Point p;
                p.x = i;
                p.y = j;
                mm.push_back(p);
            }
        }
    }
    return mm;
}

std::vector<Point> robot::getParticlePoints()
{
    std::vector<Point> points;

    for (const auto& p : potentialPositions)
    {
        if (!isInsideMap(p.x, p.y))
            continue;

        Point point;
        point.x = p.x;
        point.y = p.y;

        points.push_back(point);
    }

    return points;
}
Point robot::getBestParticlePoint()
{
    Point point;
    point.x = -1;
    point.y = -1;

    if (potentialPositions.empty())
        return point;

    point.x = lastBestParticle.x;
    point.y = lastBestParticle.y;

    return point;
}


void robot::loadMap(QString fileName)
{
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly))
        return; // handle error properly

    file.read(reinterpret_cast<char*>(map), sizeof(map));

    file.close();

    fillOutsideMazeAsWalls();

    startMonteCarlo();

    // for(int i = 0; i < 280 ;i++){
    //     for(int j = 0; j < 280 ;j++){
    //         std::cout << map[i][j];
    //     }
    //     std::cout << std::endl;
    // }


    emit resetMap();

}

void robot::saveMap()
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");

    QString fileName = "map_" + timestamp + ".bin";

    QSaveFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
        return;

    file.write(reinterpret_cast<char*>(map), sizeof(map));

    if (!file.commit()) {
        return;
    }
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

void robot::setState(int state)
{
    robot::state = state;
}
int robot::getState()
{
    return robot::state;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{

    if (!robot::initParam){
        lastValueLeft = robotdata.EncoderLeft;
        lastValueRight = robotdata.EncoderRight;

        gyroOffSet = robotdata.GyroAngle/100;

        Position startPosition;
        startPosition.x = 0;
        startPosition.y = 0;
        TimePosition startTimePosition;
        startTimePosition.angle = robotdata.GyroAngle/100- gyroOffSet;
        startTimePosition.timeStamp = (double)robotdata.synctimestamp;
        startTimePosition.pos = startPosition;
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);
        pastPositions.push_back(startTimePosition);

        // generatePotentialPositions(potentialPositions, 1000, 500, 700);

        robot::initParam = true;

    }

    TimePosition TimePosition;
    TimePosition.timeStamp = (double)robotdata.synctimestamp;
    ///tu mozete robit s datami z robota///
    double lenghtTraveled = robot::getDistanceFromWhells(realDistanceTraveled(robotdata.EncoderLeft, &lastValueLeft),realDistanceTraveled(robotdata.EncoderRight, &lastValueRight)); // m

    TimePosition.angle = robotdata.GyroAngle/100- gyroOffSet;

    double a = qDegreesToRadians(TimePosition.angle);

    TimePosition.pos.x = pastPositions.back().pos.x + lenghtTraveled * std::cos(a); //m
    TimePosition.pos.y = pastPositions.back().pos.y + lenghtTraveled * std::sin(a); //m


    pastPositions.push_back(TimePosition);

    if (state == 1 | state == 2){
        if (!position_list.empty()){
            auto target = position_list.front();

            double dis_e = robot::calculateDistanceError(target, pastPositions.back().pos.x, pastPositions.back().pos.y);

            if (0.2 > dis_e){
                forwardspeed = 0;
                rotationspeed = 0;
                position_list.pop_front();
                emit resetMap();
                emit publishWaypoints(position_list);
            } else {
                if (dis_e*2*MAX_SPEED > MAX_SPEED){
                    forwardspeed = robot::ramp(MAX_SPEED,d,forwardspeed);
                }else{
                    forwardspeed = robot::ramp(dis_e*2*MAX_SPEED,d,forwardspeed);
                }
            }

            double ang_e = normalizeAngleDeg(selectedDirection);

            if (std::abs(ang_e) < 5){
                rotationspeed = 0;
            }
            else if (std::abs(ang_e) > 45){
                forwardspeed = 0;
                if(std::abs(ang_e*0.05*MAX_SPEED_ANG) > MAX_SPEED_ANG){
                    rotationspeed =robot::ramp(sign(ang_e)*MAX_SPEED_ANG, 0.05,rotationspeed);
                }
                else{
                    rotationspeed = robot::ramp(MAX_SPEED_ANG*0.05*ang_e, 0.05,rotationspeed);
                }
            }
            else{
                if(std::abs(ang_e*0.05*MAX_SPEED_ANG) > MAX_SPEED_ANG){
                    rotationspeed = robot::ramp(sign(ang_e)*MAX_SPEED_ANG, 0.05,rotationspeed);
                }
                else{
                    rotationspeed = robot::ramp(MAX_SPEED_ANG*0.05*ang_e, 0.05,rotationspeed);
                }
            }
        }
        else{
            forwardspeed = 0;
            rotationspeed = 0;
        }
    }
    if (state == 1 && newLidarData){
        std::vector<Point> mapList;

        mapList.reserve(copyOfLaserData.size());
        for(int i = 0; i < copyOfLaserData.size();i++){
            if((copyOfLaserData.at(i).scanDistance > 0 && copyOfLaserData.at(i).scanDistance <= 500) || (copyOfLaserData.at(i).scanDistance >= 750 && copyOfLaserData.at(i).scanDistance <= 3000)){
                bool exist = false;
                double lastAngle;
                double angle;
                double lastX;
                double lastY;
                double x;
                double y;
                double lastt;
                double t;
                double it;

                for(int j = 0;j<pastPositions.size()-1;j++){
                    if((pastPositions.at(pastPositions.size()-1-j).timeStamp <= (double)copyOfLaserData.at(i).timestamp) && ((double)copyOfLaserData.at(i).timestamp <= pastPositions.at(pastPositions.size()-j).timeStamp)){
                        lastAngle = pastPositions.at(pastPositions.size()-1-j).angle;
                        angle = pastPositions.at(pastPositions.size()-j).angle;
                        lastX = pastPositions.at(pastPositions.size()-1-j).pos.x;
                        lastY = pastPositions.at(pastPositions.size()-1-j).pos.y;
                        x = pastPositions.at(pastPositions.size()-j).pos.x;
                        y = pastPositions.at(pastPositions.size()-j).pos.y;
                        lastt =pastPositions.at(pastPositions.size()-1-j).timeStamp;
                        t =pastPositions.at(pastPositions.size()-j).timeStamp;
                        it = (double)copyOfLaserData.at(i).timestamp;
                        exist =true;
                        break;
                    }
                }

                if(exist){
                    double iangle = interpolateAngle(lastAngle,angle,lastt,t,it);

                    double lidx = (copyOfLaserData.at(i).scanDistance*std::cos(qDegreesToRadians(iangle - copyOfLaserData.at(i).scanAngle)))/1000;
                    double lidy = (copyOfLaserData.at(i).scanDistance*std::sin(qDegreesToRadians(iangle - copyOfLaserData.at(i).scanAngle)))/1000;

                    double ix = interpolate(lastX, x, lastt, t, it);
                    double iy = interpolate(lastY, y, lastt, t, it);

                    Position pos;

                    pos.x = lidx + ix;
                    pos.y = lidy + iy;

                    mapList.push_back(xyToMapTransform(pos));

                    printToMap(pos);
                }
            }
        }

        emit publishMap(mapList);
        emit publishWaypoints(position_list);
        newLidarData = false;
        createCostmap = true;
    }
    if(state == 2){
        if(createCostmap){
            createCostMap(7);
            createCostmap = false;
        }
        if(navigation && !position_list.empty()){
            navigation = false;
            Position myP;
            myP = position_list.back();
            if (createPath(xyToMapTransform(myP))){
                QQueue<Position> pp;

                pp = getPathKeyPositions();


                emit resetMap();
                emit publishWaypoints(pp);

                position_list.clear();

                for(auto i: pp){
                    position_list.push_back(i);
                }
                position_list.push_back(myP);
            }
            clear_path();
        }
        if(position_list.empty()){
            navigation = true;
        }

    }

    pastPositions.erase(pastPositions.begin());

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(pastPositions.back().pos.x,pastPositions.back().pos.y,pastPositions.back().angle);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }

    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

double robot::distance_polar(double r1, double theta1, double r2, double theta2) {
    return std::sqrt(r1*r1 + r2*r2 - 2*r1*r2*cos(theta1 - theta2));
}

double robot::interpolate(double x1, double x2,
                          double t1, double t2, double t)
{
    if (t2 == t1)
        return x1;

    return x1 + (t - t1) * (x2 - x1) / (t2 - t1);
}

double robot::interpolateAngle(double a1, double a2, double t1, double t2, double t)
{
    double diff = a2 - a1;

    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    double ratio = (t - t1) / (t2 - t1);
    return a1 + ratio * diff;
}

void robot::printToMap(Position pos)
{
    int sizePx = MAP_SIZE_METERS * PIXEL_PER_METER;

    int px = pos.x * PIXEL_PER_METER;
    int py = pos.y * PIXEL_PER_METER;

    px += sizePx / 2;
    py = sizePx / 2 - py;

    if(px >= 0 && px < sizePx && py >= 0 && py < sizePx)
    {
        map[px][py] = 1;
    }
}

void robot::createCostMap(int numOfPixels)
{
    for(int k = 1; k <= numOfPixels; k++){
        for (int i = 0; i< MAP_SIZE_METERS*PIXEL_PER_METER;i++){
            for (int j = 0; j< MAP_SIZE_METERS*PIXEL_PER_METER;j++){
                Point p;
                p.x = i;
                p.y = j;
                if(map[p.x][p.y] == k){
                    std::vector<Point> fill = findElementAroundPoint(p,0);
                    for(int l =0; l < fill.size(); l++){
                        map[fill.at(l).x][fill.at(l).y]=k+1;
                    }
                }
            }
        }
    }
}

bool robot::createPath(Point p)
{
    map[p.x][p.y] = -1;
    std::vector<Point> f = findLowerThenElementAroundPoint(p,1);
    std::vector<Point> sus;
    sus.reserve(8);
    QQueue<Point> storage;

    int k = -2;
    while(!f.empty()){
        for (int i = 0; i < f.size();i++){
            sus = findElementAroundPointCross(f.at(i),0);

            for (int j = 0; j < sus.size(); j++){
                map[sus.at(j).x][sus.at(j).y] = k;
                storage.push_back(sus.at(j));
            }
        }
        f.clear();
        if(storage.empty()){
            break;
        }
        for(auto i : storage){
            f.push_back(i);
        }
        storage.clear();
        k--;
    }


    Point robotPosition = xyToMapTransform(pastPositions.back().pos);

    if(map[robotPosition.x][robotPosition.y] <= k){
        return false;
    }else{
        return true;
    }

}

void robot::clear_path()
{
    for (int i = 0; i< MAP_SIZE_METERS*PIXEL_PER_METER;i++){
        for (int j = 0; j< MAP_SIZE_METERS*PIXEL_PER_METER;j++){
            if (map[i][j] != 1){
                map[i][j] = 0;
            }
        }
    }
}

QQueue<Position> robot::getPathKeyPositions()
{
    QQueue<Position> keyPoints;
    std::vector<Point> sus;
    sus.reserve(8);
    Point start = xyToMapTransform(pastPositions.back().pos);
    Point point = start;
    Point last;
    int lastDirection = 0;

    while(1){
        last = point;
        sus = findElementAroundPoint(point, map[point.x][point.y]+1);
        if(sus.empty()){
            keyPoints.push_back(mapToXYTransform(last));
            break;
        }
        point = sus.front();
        int dir = findDirection(last,point);
        if(dir != lastDirection){
            lastDirection = dir;
            keyPoints.push_back(mapToXYTransform(point));
        }
    }
    return keyPoints;
}

std::vector<Point> robot::findElementAroundPoint(Point p, int element)
{
    std::vector<Point> result;

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            int nx = p.x + dx;
            int ny = p.y + dy;

            if (nx >= 0 && nx < MAP_SIZE_METERS*PIXEL_PER_METER && ny >= 0 && ny < MAP_SIZE_METERS*PIXEL_PER_METER)
            {
                if (map[nx][ny] == element)
                {
                    result.push_back({nx, ny});
                }
            }
        }
    }

    return result;
}

std::vector<Point> robot::findElementAroundPointCross(Point p, int element)
{
    std::vector<Point> result;

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            if(dx == 0 | dy == 0){
                int nx = p.x + dx;
                int ny = p.y + dy;

                if (nx >= 0 && nx < MAP_SIZE_METERS*PIXEL_PER_METER && ny >= 0 && ny < MAP_SIZE_METERS*PIXEL_PER_METER)
                {
                    if (map[nx][ny] == element)
                    {
                        result.push_back({nx, ny});
                    }
                }
            }
        }
    }

    return result;
}

std::vector<Point> robot::findLowerThenElementAroundPoint(Point p, int element)
{
    std::vector<Point> result;

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            int nx = p.x + dx;
            int ny = p.y + dy;

            if (nx >= 0 && nx < MAP_SIZE_METERS*PIXEL_PER_METER && ny >= 0 && ny < MAP_SIZE_METERS*PIXEL_PER_METER)
            {
                if (map[nx][ny] < element)
                {
                    result.push_back({nx, ny});
                }
            }
        }
    }

    return result;
}

int robot::findDirection(Point last, Point point)
{
    int dx = point.x - last.x;
    int dy = point.y - last.y;

    if (dx == 0 && dy == 0) return 0;

    if (dx > 0 && dy > 0) return 1; // top-right
    if (dx == 0 && dy > 0) return 2; // top
    if (dx < 0 && dy > 0) return 3; // top-left
    if (dx < 0 && dy == 0) return 4; // left
    if (dx < 0 && dy < 0) return 5; // bottom-left
    if (dx == 0 && dy < 0) return 6; // bottom
    if (dx > 0 && dy < 0) return 7; // bottom-right
    if (dx > 0 && dy == 0) return 8; // right

    return -1;
}

int robot::sign(double x)
{
    if(x > 0){
        return 1;
    }
    else if(x < 0){
        return -1;
    }
    else{
        return 0;
    }
}

Point robot::xyToMapTransform(Position pos)
{
    Point p;
    p.x = pos.x * PIXEL_PER_METER;
    p.y = pos.y * PIXEL_PER_METER;

    p.x += MAP_SIZE_METERS * PIXEL_PER_METER / 2;
    p.y = MAP_SIZE_METERS * PIXEL_PER_METER / 2 - p.y;

    return p;
}

Position robot::mapToXYTransform(Point p)
{
    Position pos;

    double centeredX = p.x - MAP_SIZE_METERS * PIXEL_PER_METER / 2;
    double centeredY = MAP_SIZE_METERS * PIXEL_PER_METER / 2 - p.y;

    pos.x = centeredX / PIXEL_PER_METER;
    pos.y = centeredY / PIXEL_PER_METER;

    return pos;
}

double robot::calculateDistanceError(Position setPoint, double x, double y){
    return std::sqrt(std::pow(setPoint.x - x,2)+std::pow(setPoint.y-y,2));
}

double robot::normalizeAngleDeg(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

double robot::calculateAngleError(Position setPoint, double x, double y, double fi) {
    double desiredAngle = std::atan2(setPoint.y - y, setPoint.x - x) * 180.0 / M_PI;
    double error = desiredAngle - fi;

    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    return error;
}
///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData=laserData;

    mclCounter++;

    if (!potentialPositions.empty() && mclCounter % 5 == 0)
    {
        monteCarloTestStep(laserData);
    }

    int sectors = binHistogram.size();
    double sectorWidth = 360.0 / sectors;

    bool nearGoal = false;
    double distanceToGoal = 999999.0;

    if (!position_list.empty() && !pastPositions.empty())
    {
        Position target = position_list.front();

        double dx = target.x - pastPositions.back().pos.x;
        double dy = target.y - pastPositions.back().pos.y;

        distanceToGoal = std::sqrt(dx * dx + dy * dy);
        nearGoal = distanceToGoal < 100.0;
    }

    std::vector<double> histogram = getHistogram(laserData,sectors,sectorWidth,nearGoal,distanceToGoal);

    updateBinHistogram(binHistogram,histogram,sectors,sectorWidth);

    std::vector<int> maskedHistogram = applyMask(binHistogram, laserData, sectors);


    // std::cout << "BIN: ";
    // for (int i = 0; i < sectors; i++)
    // {
    //     std::cout << binHistogram[i];
    // }
    // std::cout << std::endl;

    // std::cout << "MSK: ";
    // for (int i = 0; i < sectors; i++)
    // {
    //     std::cout << maskedHistogram[i];
    // }
    // std::cout << std::endl;


    std::vector<double> candidates = getCandidates(maskedHistogram,sectors);

    addGoalCandidate(candidates,maskedHistogram,sectors,sectorWidth);

    // for(int i = 0;i<candidates.size();i++){
    //     std::cout << "Kandidat " << i + 1 << " je " << candidates[i] << " stupnov." << std::endl;
    // }

    if (!position_list.empty() && !candidates.empty())
    {
        Position target = position_list.front();

        double goalDirection = robot::calculateAngleError(target, pastPositions.back().pos.x, pastPositions.back().pos.y, pastPositions.back().angle);
        if (goalDirection < 0.0) goalDirection += 360.0;

        double currentDirection = 0.0; // rovno pred robot
        double previousDirection = previousSelectedDirection;
        if (previousDirection < 0.0) previousDirection += 360.0;

        selectedDirection = selectBestCandidate(candidates,
                                                goalDirection,
                                                currentDirection,
                                                previousDirection);

        previousSelectedDirection = selectedDirection;
    }

    // std::cout << "selectedDirection = " << selectedDirection << std::endl;



    newLidarData = true;
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   //update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

double robot::ramp(double target, double d, double y){

    if(target-0.1 > y){
        y += d;
    }
    else if(target+0.1 < y){
        y -= d;
    }
    else{
        y = target;
    }

    return y;
}



double robot::getDistanceFromWhells(double leftWheel, double rightWheel)
{
    return (leftWheel+rightWheel)/2;
}

double robot::realDistanceTraveled(unsigned short encoderValue, unsigned short *LastValue)
{
    unsigned short last = *LastValue;

    int32_t diff = (int32_t)encoderValue - (int32_t)last;

    *LastValue = encoderValue;

    if (diff > 32767)
        diff -= 65536;
    else if (diff < -32768)
        diff += 65536;

    return (double)diff*TICK_TO_METER;
}

std::vector<double> robot::getCandidates(std::vector<int> binHistogram,int sectors)
{
    std::vector<double> candidates;
    double candidate;
    double sectorWidth = 360.0 / sectors;
    std::vector<FreeInterval> intervals = getFreeIntervals(binHistogram,sectors);
    int width;
    for(auto i:intervals){
        width = getIntervalWidth(i,sectors);
        if(width < 5){
            candidate = sectorWidth * width / 2 + sectorWidth * i.start;
            if(candidate >= 360) candidate -= 360;
            candidates.push_back(candidate);
        }else{
            candidate = i.start * sectorWidth + sectorWidth;
            while(candidate >= 360.0) candidate -= 360.0;
            while(candidate < 0.0) candidate += 360.0;
            candidates.push_back(candidate);
            candidate = i.end * sectorWidth - sectorWidth;
            while(candidate >= 360.0) candidate -= 360.0;
            while(candidate < 0.0) candidate += 360.0;
            candidates.push_back(candidate);
        }
    }
    return candidates;
}

void robot::addGoalCandidate(std::vector<double>& candidates, std::vector<int>& maskedHistogram, int sectors, double sectorWidth){
    if (!position_list.empty())
    {
        Position target = position_list.front();

        double goalCandidate = robot::calculateAngleError(target, pastPositions.back().pos.x, pastPositions.back().pos.y, pastPositions.back().angle);

        if (goalCandidate < 0.0)
            goalCandidate += 360.0;

        int goalSector = int(goalCandidate / sectorWidth);
        if (goalSector >= sectors) goalSector = sectors - 1;

        if (maskedHistogram[goalSector] == 0)
        {
            bool alreadyThere = false;

            for (double c : candidates)
            {
                if (std::abs(c - goalCandidate) < 0.001)
                {
                    alreadyThere = true;
                    break;
                }
            }

            if (!alreadyThere)
                candidates.push_back(goalCandidate);
        }
    }
}

std::vector<double> robot::getHistogram(const std::vector<LaserData>& laserData,int sectors, double sectorWidth, bool nearGoal, double distanceToGoal){

    std::vector<double> histogram(sectors,0.0);

    double mi;
    double ci = 1;
    double a = 10;
    double b = 0.01;

    int CurSector = 0;

    double safeRadius = 150.0 + 80.0;

    for (auto &p : laserData)
    {

        double angle = 360.0 - p.scanAngle; // 1 - 360
        if(angle == 360.0) angle = 0; // 0 - 359
        double dist  = p.scanDistance; // mm

        if (dist <= 0.0) continue;
        // if (nearGoal && dist > distanceToGoal + 50.0)
        //     continue;

        //std::cout << "Uhol: " << angle << " Vzdialenost: " << dist << std::endl;

        mi = std::pow(ci,2)*(a-b*dist);
        if (mi < 0) mi = 0;

        double gamma;

        if (dist <= safeRadius)
        {
            gamma = 90.0;
        }
        else
        {
            gamma = qRadiansToDegrees(std::asin(safeRadius / dist));
        }

        double strAngle = angle - gamma;
        double endAngle = angle + gamma;

        while (strAngle < 0.0) strAngle += 360.0;
        while (strAngle >= 360.0) strAngle -= 360.0;
        while (endAngle < 0.0) endAngle += 360.0;
        while (endAngle >= 360.0) endAngle -= 360.0;

        int strSector = int(strAngle/sectorWidth);
        int endSector = int(endAngle/sectorWidth);

        if (strSector <= endSector)
        {
            for (int s = strSector; s <= endSector; s++)
            {
                histogram[s] += mi;
            }
        }
        else
        {
            for (int s = strSector; s < sectors; s++)
            {
                histogram[s] += mi;
            }
            for (int s = 0; s <= endSector; s++)
            {
                histogram[s] += mi;
            }
        }

        // CurSector = int(angle/sectorWidth);
        // if(CurSector == 36) CurSector = 0;
        // histogram[CurSector] += mi;

    }

    // for (int i = 0; i < sectors; i++)
    // {
    //     std::cout << "Sektor " << i
    //               << " [" << i*sectorWidth << ", " << (i+1)*sectorWidth << ") : "
    //               << histogram[i] << std::endl;

    // }
    return histogram;
}

void robot::updateBinHistogram(std::vector<int>& binHistogram,std::vector<double>& histogram,int sectors, double sectorWidth){
    //zohladni zmenu natocenia aby krajova oblast nebola zle shiftnuta
    int upperLimit = 50;//90 - 50
    int lowerLimit = 30;//70 - 30

    double angleDiff = 0.0;

    if (pastPositions.size() >= 2)
    {
        double currentAngle = pastPositions.back().angle;
        double previousAngle = pastPositions[pastPositions.size() - 2].angle;

        angleDiff = currentAngle - previousAngle;

        while (angleDiff > 180.0) angleDiff -= 360.0;
        while (angleDiff < -180.0) angleDiff += 360.0;

        // std::cout << "Zmena uhla: " << angleDiff << std::endl;
    }

    int histBinShift = static_cast<int>(std::round(angleDiff / sectorWidth));

    if (std::abs(histBinShift) > 0)
    {
        int j = histBinShift;

        std::vector<int> shiftedBinHistogram;
        shiftedBinHistogram.assign(sectors, 0);

        for (int i = 0; i < sectors; i++)
        {
            if (j < 0) j += sectors;
            if (j >= sectors) j -= sectors;

            shiftedBinHistogram[i] = binHistogram[j];

            j++;
        }

        binHistogram = shiftedBinHistogram;
    }

    for (int i = 0; i < sectors; i++)
    {
        if (histogram[i] >= upperLimit)
        {
            binHistogram[i] = 1;
        }
        else if (histogram[i] <= lowerLimit)
        {
            binHistogram[i] = 0;
        }
    }
}

std::vector<int> robot::applyMask(const std::vector<int>& binHistogram,
                                  const std::vector<LaserData>& laserData,
                                  int sectors)
{
    std::vector<int> maskedHistogram = binHistogram;

    double sectorWidth = 360.0 / sectors;

    double minTurnRadius = 300.0;   // mm Treba spravit dynamicky tak aby sa mozna draha vzhladom na aktualny dyn stav robota
    double robotRadius   = 180.0;   // mm
    // double safetyMargin  = 40.0;    // mm

    //Treba vyblokovat vsetky smery pod max natocenim -> blokovany sektor 10 vyblokuj 10 az 18 rovnako na pravo

    double forbiddenBand = robotRadius;

    for (const auto &p : laserData)
    {
        double angleDeg = 360.0 - p.scanAngle;
        if (angleDeg >= 360.0) angleDeg -= 360.0;

        double dist = p.scanDistance;
        if (dist <= 0.0) continue;

        double angleRad = qDegreesToRadians(angleDeg);

        double px = dist * std::cos(angleRad);
        double py = dist * std::sin(angleRad);

        double dLeft  = std::sqrt(pow(px,2) + pow((py - minTurnRadius),2));
        double dRight = std::sqrt(pow(px,2) + pow((py + minTurnRadius),2));

        int obstacleSector = int(angleDeg / sectorWidth);
        if (obstacleSector >= sectors) obstacleSector = sectors - 1;

        if (py > 0.0)
        {
            double err = std::abs(dLeft - minTurnRadius);

            if (err <= forbiddenBand)
            {
                for (int k = 0; k < sectors; k++)
                {
                    int s = obstacleSector + k;
                    while (s >= sectors) s -= sectors;

                    double centerAngle = s * sectorWidth + sectorWidth / 2.0;
                    double relAngle = centerAngle;
                    if (relAngle > 180.0)
                        relAngle -= 360.0;

                    if (relAngle <= 0.0)
                        break;

                    maskedHistogram[s] = 1;
                }
            }
        }

        if (py < 0.0)
        {
            double err = std::abs(dRight - minTurnRadius);

            if (err <= forbiddenBand)
            {
                for (int k = 0; k < sectors; k++)
                {
                    int s = obstacleSector - k;
                    while (s < 0) s += sectors;

                    double centerAngle = s * sectorWidth + sectorWidth / 2.0;
                    double relAngle = centerAngle;
                    if (relAngle > 180.0)
                        relAngle -= 360.0;

                    if (relAngle >= 0.0)
                        break;

                    maskedHistogram[s] = 1;
                }
            }
        }
    }
    return maskedHistogram;
}

std::vector<FreeInterval> robot::getFreeIntervals(const std::vector<int>& binHistogram,int sectors)
{
    bool inInterval = false;
    std::vector<FreeInterval> intervals;
    FreeInterval p;
    for(int h = 0; h < binHistogram.size();h++)
    {
        if(binHistogram[h] == 0 && inInterval == false)
        {
            p.start = h;
            inInterval = true;
        }
        else if(binHistogram[h] == 1 && inInterval == true){
            p.end = h - 1;
            inInterval = false;
            intervals.push_back(p);
        }
    }
    if(inInterval){
        p.end = sectors - 1;
        intervals.push_back(p);
    }
    if(intervals.size() > 1 && intervals.front().start == 0 && intervals.back().end == sectors - 1){
        p.start = intervals.back().start;
        p.end = intervals.front().end;
        intervals.erase(intervals.begin());
        intervals.pop_back();
        intervals.push_back(p);
    }
    return intervals;
}

int robot::getIntervalWidth(const FreeInterval& interval, int sectors)
{
    if (interval.end >= interval.start)
        return interval.end - interval.start + 1;
    else
        return (sectors - interval.start) + (interval.end + 1);
}

double robot::angleDiffDeg(double a, double b)
{
    double diff = std::abs(a - b);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

double robot::selectBestCandidate(const std::vector<double>& candidates,
                                  double goalDirection,
                                  double currentDirection,
                                  double previousDirection)
{
    if (candidates.empty())
        return currentDirection;

    double wg = 2.0;   // ciel
    double wc = 1.0;   // aktualny smer
    double wp = 0.8;   // predchadzajuci vyber

    double bestCandidate = candidates[0];
    double bestCost = 1e9;

    for (double c : candidates)
    {
        double cost =
            wg * angleDiffDeg(c, goalDirection) +
            wc * angleDiffDeg(c, currentDirection) +
            wp * angleDiffDeg(c, previousDirection);

        // std::cout << "cost = "<< wg * angleDiffDeg(c, goalDirection) <<" + " << wc * angleDiffDeg(c, currentDirection)<<" + "<< wp * angleDiffDeg(c, previousDirection) <<" = " << cost <<std::endl;

        if (cost < bestCost)
        {
            bestCost = cost;
            bestCandidate = c;
        }
    }

    return bestCandidate;
}

void robot::generatePotentialPositions(std::vector<PotentialPosition>& potentialPositions, int mapWidth, int mapHeight, int numOfPositions){
    for(int i = 0; i < numOfPositions; i++){

        PotentialPosition pos;
        do{
        pos.x = rand() % (mapWidth);
        pos.y = rand() % (mapHeight);
        }while(map [pos.x][pos.y] != 0);
        pos.phi = rand() % (361) - 180; // -180 - 180
        pos.weight = 1.0 / numOfPositions;
        pos.error = 0.0;
        pos.isDebug = false;
        potentialPositions.push_back(pos);
    }

    // int j = 0;
    // for (auto i: potentialPositions) {
    //     std::cout << "Kandidat " << j++ << ": " << std::endl;
    //     std::cout <<"Suradnica x =" << i.x << std::endl;
    //     std::cout <<"Suradnica y =" << i.y << std::endl;
    //     std::cout <<"Orientacia phi =" << i.phi << std::endl;
    //     std::cout <<"Vaha =" << i.weight << std::endl;
    // }
}

double robot::rayTraceBresenham(const PotentialPosition& pos, double laserAngleDeg)
{

    int mapSize = MAP_SIZE_METERS * PIXEL_PER_METER;

    double globalAngleDeg = (pos.phi - 90) - laserAngleDeg;
    double angleRad = globalAngleDeg * M_PI / 180.0;

    int maxDistancePx = 240;

    int x0 = pos.x;
    int y0 = pos.y;

    int x1 = x0 + cos(angleRad) * maxDistancePx;
    int y1 = y0 - sin(angleRad) * maxDistancePx;

    // if (pos.x == 140 && pos.y == 140 && pos.phi == 90 &&
    //     (laserAngleDeg == 0 || laserAngleDeg == 90 || laserAngleDeg == 180 || laserAngleDeg == 270))
    // {
    //     std::cout << "RAY DEBUG " << "laserAngle=" << laserAngleDeg << " globalAngle=" << globalAngleDeg << " start=(" << x0 << "," << y0 << ")" << " end=(" << x1 << "," << y1 << ")" << std::endl;
    // }

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true)
    {
        if (x < 0 || x >= mapSize || y < 0 || y >= mapSize)
        {
            double dist = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
            return dist;
        }

        if (map[x][y] == 1)
        {
            double dist = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
            return dist;
        }

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return maxDistancePx;
}
double robot::calculateParticleError(const PotentialPosition& pos, const std::vector<LaserData>& laserData)
{
    double error = 0.0;
    int usedRays = 0;


    for (int k = 0; k < laserData.size(); k += 5)
    {
        double distMm = laserData[k].scanDistance;

        if (distMm < 100)continue;

        if (distMm > 10000)continue;

        double realDistancePx = laserData[k].scanDistance / 1000.0 * PIXEL_PER_METER;

        double simulatedDistancePx = rayTraceBresenham(pos, laserData[k].scanAngle);

        double diff = realDistancePx - simulatedDistancePx;

        error += diff * diff;
        usedRays++;

        // // DEBUG: vypis iba pre predpokladanu skutocnu polohu robota
        // if (pos.x == 140 && pos.y == 140 && pos.phi == 90)
        // {
        //     static int debugRayCounter = 0;

        //     if (debugRayCounter < 40)
        //     {
        //         std::cout << "DEBUG RAY " << "angle=" << laserData[k].scanAngle << " realPx=" << realDistancePx << " simPx=" << simulatedDistancePx << " diff=" << diff << " diff2=" << diff * diff << std::endl;

        //         debugRayCounter++;
        //     }
        // }
        // // DEBUG: vypis iba pre predpokladanu skutocnu polohu robota
    }

    if (usedRays == 0)
        return 1e9;

    // if (pos.x == 140 && pos.y == 140 && pos.phi == 90)
    // {
    //     std::cout << "DEBUG TRUE PARTICLE AVG ERROR = "
    //               << error / usedRays
    //               << " usedRays=" << usedRays
    //               << std::endl;
    // }


    return error / usedRays;
}
double robot::errorToWeight(double error)
{
    return 1.0 / (1.0 + error);
}

void robot::updateWeights(std::vector<PotentialPosition>& potentialPositions, const std::vector<LaserData>& laserData){
    if (potentialPositions.empty())
        return;

    double weightSum = 0.0;

    for (auto& pos : potentialPositions)
    {
        double error = calculateParticleError(pos, laserData);
        pos.error = error;
        pos.weight = errorToWeight(error);

        weightSum += pos.weight;
    }

    if (weightSum == 0.0)
    {
        double uniformWeight = 1.0 / potentialPositions.size();

        for (auto& pos : potentialPositions)
        {
            pos.weight = uniformWeight;
        }

        return;
    }

    for (auto& pos : potentialPositions)
    {
        pos.weight /= weightSum;
    }
}


bool robot::isInsideMap(int x, int y)
{
    int size = MAP_SIZE_METERS * PIXEL_PER_METER;
    return x >= 0 && x < size && y >= 0 && y < size;
}

void robot::fillOutsideMazeAsWalls()
{
    int size = MAP_SIZE_METERS * PIXEL_PER_METER;

    QQueue<Point> queue;

    // 1. Do fronty vlozime vsetky volne bunky na okrajoch mapy
    for (int i = 0; i < size; i++)
    {
        // horny okraj
        if (map[i][0] == 0)
        {
            map[i][0] = -99;
            queue.push_back({i, 0});
        }

        // dolny okraj
        if (map[i][size - 1] == 0)
        {
            map[i][size - 1] = -99;
            queue.push_back({i, size - 1});
        }

        // lavy okraj
        if (map[0][i] == 0)
        {
            map[0][i] = -99;
            queue.push_back({0, i});
        }

        // pravy okraj
        if (map[size - 1][i] == 0)
        {
            map[size - 1][i] = -99;
            queue.push_back({size - 1, i});
        }
    }

    // 2. Flood-fill cez 4-susednost
    int dx[4] = { 1, -1, 0, 0 };
    int dy[4] = { 0, 0, 1, -1 };

    while (!queue.empty())
    {
        Point current = queue.front();
        queue.pop_front();

        for (int k = 0; k < 4; k++)
        {
            int nx = current.x + dx[k];
            int ny = current.y + dy[k];

            if (!isInsideMap(nx, ny))
                continue;

            if (map[nx][ny] == 0)
            {
                map[nx][ny] = -99;
                queue.push_back({nx, ny});
            }
        }
    }

    // 3. Vsetky bunky dosiahnutelne zvonku prepiseme na steny
    for (int x = 0; x < size; x++)
    {
        for (int y = 0; y < size; y++)
        {
            if (map[x][y] == -99)
            {
                map[x][y] = 1;
            }
        }
    }
}

void robot::resampleParticles(std::vector<PotentialPosition>& potentialPositions)
{
    std::vector<PotentialPosition> newParticles;

    int numberOfParticles = potentialPositions.size();

    if (numberOfParticles == 0)
        return;

    for (int i = 0; i < numberOfParticles; i++)
    {
        double r = (double)rand() / RAND_MAX;
        double cumulativeWeight = 0.0;

        for (const auto& particle : potentialPositions)
        {
            cumulativeWeight += particle.weight;

            if (cumulativeWeight >= r)
            {
                PotentialPosition selected = particle;

                selected.weight = 1.0 / numberOfParticles;

                newParticles.push_back(selected);
                break;
            }
        }
    }
    while (newParticles.size() < numberOfParticles)
    {
        PotentialPosition selected = potentialPositions.back();
        selected.weight = 1.0 / numberOfParticles;
        newParticles.push_back(selected);
    }

    potentialPositions = newParticles;
}

void robot::addNoiseToParticle(PotentialPosition& p)
{
    int oldX = p.x;
    int oldY = p.y;

    int positionNoise = 2;   // pixely
    int angleNoise = 5;      // stupne

    p.x += (rand() % (2 * positionNoise + 1)) - positionNoise;
    p.y += (rand() % (2 * positionNoise + 1)) - positionNoise;
    p.phi += (rand() % (2 * angleNoise + 1)) - angleNoise;

    if (!isFreeCell(p.x, p.y))
    {
        p.x = oldX;
        p.y = oldY;
    }

    if (p.phi > 180) p.phi -= 360;
    if (p.phi < -180) p.phi += 360;
}

bool robot::isFreeCell(int x, int y)
{
    int mapSize = MAP_SIZE_METERS * PIXEL_PER_METER;

    if (x < 0 || x >= mapSize || y < 0 || y >= mapSize)
        return false;

    return map[x][y] == 0;
}
PotentialPosition robot::getBestParticle()
{

    if (potentialPositions.empty())
    {
        return {0, 0, 0.0, 0.0};
    }

    PotentialPosition best = potentialPositions[0];

    for (const auto& p : potentialPositions)
    {
        if (p.weight > best.weight)
        {
            best = p;
        }
    }

    return best;
}

void robot::initMonteCarloLocalization()
{
    potentialPositions.clear();

    int mapWidth = MAP_SIZE_METERS * PIXEL_PER_METER;
    int mapHeight = MAP_SIZE_METERS * PIXEL_PER_METER;

    int numOfParticles = 2000;
    lastRobotPoseInitialized = false;

    generatePotentialPositions(potentialPositions,mapWidth,mapHeight,numOfParticles);

    // addDebugParticle(140, 140, 90);
    // addDebugParticle(139, 139, 90);

    std::cout << "MCL initialized: " << potentialPositions.size() << " particles" << std::endl;

}

void robot::monteCarloTestStep(const std::vector<LaserData>& laserData)
{
    // clearParticleVisualization();

    if (potentialPositions.empty())
    {
        initMonteCarloLocalization();
    }

    moveParticlesByOdometry();

    updateWeights(potentialPositions, laserData);
    // printBestErrors(10);

    PotentialPosition best = getBestParticle();

    lastBestParticle = best;
    hasLastBestParticle = true;

    overwriteOdometryFromMonteCarlo(best);

    // std::cout << "MCL best before resampling: " << "x=" << best.x << " y=" << best.y << " phi=" << best.phi << " weight=" << best.weight << std::endl;

    if (!pastPositions.empty())
    {
        Point realMapPoint = xyToMapTransform(pastPositions.back().pos);

        double realPhi = normalizeAngleDeg(pastPositions.back().angle + 90.0);

        std::cout << " MCL VS REAL " << std::endl;
        std::cout << "MCL:  " << "x=" << best.x << " y=" << best.y << " phi=" << best.phi << " error=" << best.error << " weight=" << best.weight << std::endl;
        std::cout << "REAL: " << "x=" << realMapPoint.x << " y=" << realMapPoint.y << " phi=" << realPhi << std::endl;
        std::cout << "DIFF: " << "dx=" << best.x - realMapPoint.x << " dy=" << best.y - realMapPoint.y << " dPhi=" << normalizeAngleDeg(best.phi - realPhi) << std::endl;
    }

    // drawParticlesToMap();

    emit resetMap();


    resampleCounter++;

    if (resampleCounter % 1 == 0)
    {
        resampleParticles(potentialPositions);

        int randomCount = potentialPositions.size() * 0.001;

        for (int i = 0; i < randomCount && !potentialPositions.empty(); i++)
        {
            potentialPositions.pop_back();
        }


        int mapSize = MAP_SIZE_METERS * PIXEL_PER_METER;

        generatePotentialPositions(potentialPositions, mapSize, mapSize, randomCount);
    }
}

void robot::startMonteCarlo()
{
    potentialPositions.clear();

    initMonteCarloLocalization();

    mclEnabled = true;
    mclCounter = 0;

    std::cout << "Monte Carlo localization started." << std::endl;
}

void robot::clearParticleVisualization()
{
    int size = MAP_SIZE_METERS * PIXEL_PER_METER;

    for (int x = 0; x < size; x++)
    {
        for (int y = 0; y < size; y++)
        {
            if (map[x][y] == 7 || map[x][y] == 9)
            {
                map[x][y] = 0;
            }
        }
    }
}

void robot::drawParticlesToMap()
{
    if (potentialPositions.empty())
        return;

    for (const auto& p : potentialPositions)
    {
        if (!isInsideMap(p.x, p.y))
            continue;

        if (map[p.x][p.y] == 0)
        {
            map[p.x][p.y] = 7;
        }
    }

    PotentialPosition best = getBestParticle();

    if (isInsideMap(best.x, best.y))
    {
        map[best.x][best.y] = 9;
    }
}

void robot::addDebugParticle(int x, int y, double phi)
{
    PotentialPosition p;

    p.x = x;
    p.y = y;
    p.phi = phi;
    p.weight = 1.0;
    p.error = 0.0;
    p.isDebug = true;

    potentialPositions.push_back(p);

    std::cout << "Debug particle added: "
              << "x=" << x
              << " y=" << y
              << " phi=" << phi
              << std::endl;
}

void robot::printBestErrors(int count)
{
    std::vector<PotentialPosition> sorted = potentialPositions;

    std::sort(sorted.begin(), sorted.end(),
              [](const PotentialPosition& a, const PotentialPosition& b)
              {
                  return a.error < b.error;
              });

    std::cout << "----- TOP PARTICLES BY ERROR -----" << std::endl;

    for (int i = 0; i < count && i < sorted.size(); i++)
    {
        std::cout << i
                  << ": x=" << sorted[i].x
                  << " y=" << sorted[i].y
                  << " phi=" << sorted[i].phi
                  << " error=" << sorted[i].error
                  << " weight=" << sorted[i].weight;

                    if(sorted[i].isDebug)
                    {
                        std::cout << "  <--- DEBUG PARTICLE";
                    }
                  std::cout << std::endl;
    }
}

void robot::moveParticlesByOdometry()
{

    if (pastPositions.empty())
        return;

    Point currentMapPoint = xyToMapTransform(pastPositions.back().pos);

    double currentRobotX = currentMapPoint.x;
    double currentRobotY = currentMapPoint.y;

    double currentRobotPhi = normalizeAngleDeg(pastPositions.back().angle + 90.0);

    if (!lastRobotPoseInitialized)
    {
        lastRobotX = currentRobotX;
        lastRobotY = currentRobotY;
        lastRobotPhi = currentRobotPhi;
        lastRobotPoseInitialized = true;
        return;
    }

    double dx = currentRobotX - lastRobotX;
    double dy = currentRobotY - lastRobotY;

    double distance = sqrt(dx * dx + dy * dy);
    double dPhi = normalizeAngleDeg(currentRobotPhi - lastRobotPhi);

    // std::cout << "ODOM MOVE: " << "dx=" << dx << " dy=" << dy << " distance=" << distance << " dPhi=" << dPhi << " currentPhi=" << currentRobotPhi << std::endl;


    if (distance < 0.5 && std::abs(dPhi) < 1.0)
    {
        lastRobotX = currentRobotX;
        lastRobotY = currentRobotY;
        lastRobotPhi = currentRobotPhi;
        return;
    }

    double translationThreshold = 1.0;

    if (std::abs(dPhi) > 2.0)
    {
        translationThreshold = 3.0;
    }

    for (auto& p : potentialPositions)
    {
        // rotacia
        if (std::abs(dPhi) >= 1.0)
        {
            p.phi = normalizeAngleDeg(p.phi + dPhi);

            int angleNoise = 2;
            p.phi = normalizeAngleDeg(p.phi + ((rand() % (2 * angleNoise + 1)) - angleNoise));
        }

        // translacia
        if (distance >= translationThreshold)
        {
            double angleRad = (p.phi - 90.0) * M_PI / 180.0;

            p.x += cos(angleRad) * distance;
            p.y -= sin(angleRad) * distance;

            int posNoise = 2;

            p.x += (rand() % (2 * posNoise + 1)) - posNoise;
            p.y += (rand() % (2 * posNoise + 1)) - posNoise;
        }

        if (!isInsideMap(p.x, p.y) || map[p.x][p.y] != 0)
        {
            p.weight = 0.0;
        }
    }

    lastRobotX = currentRobotX;
    lastRobotY = currentRobotY;
    lastRobotPhi = currentRobotPhi;
}

void robot::overwriteOdometryFromMonteCarlo(const PotentialPosition& best)
{
    if (pastPositions.empty())
        return;

    // Prahy na ladenie
    double maxAcceptedError = 10000.0;
    double maxPositionJumpPx = 10000.0; // 25 px = 1.25 m pri 20 px/m
    double maxPhiJumpDeg = 180.0;

    Point odomMapPoint = xyToMapTransform(pastPositions.back().pos);
    double odomPhiMcl = normalizeAngleDeg(pastPositions.back().angle + 90.0);

    double dx = best.x - odomMapPoint.x;
    double dy = best.y - odomMapPoint.y;
    double positionDiff = sqrt(dx * dx + dy * dy);

    double phiDiff = std::abs(normalizeAngleDeg(best.phi - odomPhiMcl));

    bool errorIsGood = best.error < maxAcceptedError;
    bool jumpIsReasonable = positionDiff < maxPositionJumpPx && phiDiff < maxPhiJumpDeg;

    std::cout << "MCL FILTER DEBUG: "
              << "best.error=" << best.error
              << " maxAcceptedError=" << maxAcceptedError
              << " errorIsGood=" << errorIsGood
              << " positionDiff=" << positionDiff
              << " maxPositionJumpPx=" << maxPositionJumpPx
              << " phiDiff=" << phiDiff
              << " maxPhiJumpDeg=" << maxPhiJumpDeg
              << " jumpIsReasonable=" << jumpIsReasonable
              << std::endl;

    if (!errorIsGood || !jumpIsReasonable)
    {
        std::cout << "MCL ODOM OVERWRITE REJECTED: "
                  << "error=" << best.error
                  << " posDiff=" << positionDiff
                  << " phiDiff=" << phiDiff
                  << std::endl;
        return;
    }

    Point bestPoint;
    bestPoint.x = best.x;
    bestPoint.y = best.y;

    Position correctedPos = mapToXYTransform(bestPoint);

    pastPositions.back().pos = correctedPos;
    pastPositions.back().angle = normalizeAngleDeg(best.phi - 90.0);

    Point correctedMapPoint = xyToMapTransform(pastPositions.back().pos);

    lastRobotX = correctedMapPoint.x;
    lastRobotY = correctedMapPoint.y;
    lastRobotPhi = normalizeAngleDeg(pastPositions.back().angle + 90.0);
    lastRobotPoseInitialized = true;

    createCostmap = true;
    navigation = true;

    std::cout << "MCL ODOM OVERWRITE ACCEPTED: "
              << "x=" << correctedPos.x
              << " y=" << correctedPos.y
              << " angle=" << pastPositions.back().angle
              << " error=" << best.error
              << std::endl;
}



  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
