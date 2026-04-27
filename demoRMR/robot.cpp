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
            if (map[i][j] == 1){
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

            if (0.1 > dis_e){
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
            else if (std::abs(ang_e) > 90){
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
            createCostMap(9);
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

    int sectors = binHistogram.size();
    double sectorWidth = 360.0 / sectors;

    std::vector<double> histogram = getHistogram(laserData,sectors,sectorWidth);

    updateBinHistogram(binHistogram,histogram,sectors);

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

    for(int i = 0;i<candidates.size();i++){
        std::cout << "Kandidat " << i + 1 << " je " << candidates[i] << " stupnov." << std::endl;
    }

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

    std::cout << "selectedDirection = " << selectedDirection << std::endl;



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

std::vector<double> robot::getHistogram(const std::vector<LaserData>& laserData,int sectors, double sectorWidth){

    std::vector<double> histogram(sectors,0.0);

    double mi;
    double ci = 1;
    double a = 10;
    double b = 0.01;

    int CurSector = 0;

    double safeRadius = 150.0 + 50.0;

    for (auto &p : laserData)
    {

        double angle = 360.0 - p.scanAngle; // 1 - 360
        if(angle == 360.0) angle = 0; // 0 - 359
        double dist  = p.scanDistance; // mm

        if (dist <= 0.0) continue;

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

void robot::updateBinHistogram(std::vector<int>& binHistogram,std::vector<double>& histogram,int sectors){
    //zohladni zmenu natocenia aby krajova oblast nebola zle shiftnuta
    int upperLimit = 90;
    int lowerLimit = 70;
    for (int i = 0; i < sectors; i++)
    {
        if(histogram[i] >= upperLimit) binHistogram[i] = 1;
        else if(histogram[i] <= lowerLimit) binHistogram[i] = 0;
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
                int extra = 1;

                if (err < forbiddenBand * 0.66) extra = 2;
                if (err < forbiddenBand * 0.33) extra = 3;

                for (int k = 0; k <= extra; k++)
                {
                    int s = obstacleSector + k;
                    while (s >= sectors) s -= sectors;

                    double centerAngle = s * sectorWidth + sectorWidth / 2.0;
                    double relAngle = centerAngle;
                    if (relAngle > 180.0) relAngle -= 360.0;

                    if (relAngle > 0.0)
                        maskedHistogram[s] = 1;
                }
            }
        }

        if (py < 0.0)
        {
            double err = std::abs(dRight - minTurnRadius);

            if (err <= forbiddenBand)
            {
                int extra = 1;

                if (err < forbiddenBand * 0.66) extra = 2;
                if (err < forbiddenBand * 0.33) extra = 3;

                for (int k = 0; k <= extra; k++)
                {
                    int s = obstacleSector - k;
                    while (s < 0) s += sectors;

                    double centerAngle = s * sectorWidth + sectorWidth / 2.0;
                    double relAngle = centerAngle;
                    if (relAngle > 180.0) relAngle -= 360.0;

                    if (relAngle < 0.0)
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

        std::cout << "cost = "<< wg * angleDiffDeg(c, goalDirection) <<" + " << wc * angleDiffDeg(c, currentDirection)<<" + "<< wp * angleDiffDeg(c, previousDirection) <<" = " << cost <<std::endl;

        if (cost < bestCost)
        {
            bestCost = cost;
            bestCandidate = c;
        }
    }

    return bestCandidate;
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
