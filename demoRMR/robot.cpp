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
    dt = 20; // s
    forwardspeed=0;
    rotationspeed=0;
    state = 0;

    // for (int i = 0; i< 140;i++){
    //     for (int j = 0; j< 140;j++){
    //         map[i][j] = false;
    //     }
    // }

    Position p1;
    p1.x = 0;
    p1.y = 3.1;
    position_list.push(p1);

    Position p2;
    p2.x = 3.0;
    p2.y = 3.1;
    position_list.push(p2);

    Position p3;
    p3.x = 3.0;
    p3.y = 0;
    position_list.push(p3);

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

    if (!position_list.empty()){
        auto target = position_list.front();

        double dis_e = robot::calculateDistanceError(target, pastPositions.back().pos.x, pastPositions.back().pos.y);

        if (0.01 > dis_e){
            forwardspeed = 0;
            rotationspeed = 0;
            position_list.pop();
        } else {
            if (dis_e*2*MAX_SPEED > MAX_SPEED){
                forwardspeed = robot::ramp(MAX_SPEED,dt,forwardspeed);
            }else{
                forwardspeed = robot::ramp(dis_e*2*MAX_SPEED,dt,forwardspeed);
            }
        }

        double ang_e = robot::calculateAngleError(target, pastPositions.back().pos.x, pastPositions.back().pos.y, pastPositions.back().angle);

        if (std::abs(ang_e) < 5){
            rotationspeed = 0;
        }
        else if (std::abs(ang_e) > 45){
            forwardspeed = 0;
            rotationspeed = 0.01*ang_e;
        }
        else{
            rotationspeed = 0.01*ang_e;
        }
    }
    else{
        forwardspeed = 0;
        rotationspeed = 0;
    }

    if (state == 1){
        for(int i = 0; i < copyOfLaserData.size();i++){
            if (copyOfLaserData.at(i).scanDistance > 0){

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

                    lidx = lidx + ix;
                    lidy = lidy + iy;

                    emit publishMapPoint(lidx,lidy);

                    //map[lix][liy] = 1;
                }
            }
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
    // if(datacounter%50==0){
    //     emit publishMapPoint(x,y);
    // }

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

double robot::calculateDistanceError(Position setPoint, double x, double y){
    return std::sqrt(std::pow(setPoint.x - x,2)+std::pow(setPoint.y-y,2));
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

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

double robot::ramp(double target, double dt, double speed){

    if(target-0.1 > speed){
        speed += dt;
    }
    else{
        speed = target;
    }

    return speed;
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
