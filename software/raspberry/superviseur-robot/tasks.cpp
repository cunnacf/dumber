/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 50
#define PRIORITY_TOPENCOMROBOT 40
#define PRIORITY_TMOVE 33
#define PRIORITY_TSENDTOMON 35
#define PRIORITY_TRECEIVEFROMMON 34
#define PRIORITY_TSTARTROBOT 33
#define PRIORITY_TCAMERA 30
#define PRIORITY_TCAMERA_OFF 32
#define PRIORITY_TCAMERA_ON 31
#define PRIORITY_TBATTERY 33
#define PRIORITY_TCONNEXIONTOROBOTLOST 41

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startBattery, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_sendImageTask, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopSendImageTask, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findArenaCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
   
    }
    if (err = rt_sem_create(&sem_confirmFindArenaCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
 
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_getBattery, "th_getBattery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TCAMERA_ON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } 
    
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCAMERA_OFF, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImageTask, "th_sendImageTask", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_findArenaCamera, "th_findArenaCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } //y
    if (err = rt_task_create(&th_connexionRobotLost, "th_connexionRobotLost", 0, PRIORITY_TCONNEXIONTOROBOTLOST, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_getBattery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CloseCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImageTask, (void(*)(void*)) & Tasks::SendImageTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_start(&th_findArenaCamera, (void(*)(void*)) & Tasks::FindArenaCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_connexionRobotLost, (void(*)(void*)) & Tasks::ConnexionRobotLost, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}


/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    int status ;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << "Connection perdue avec le moniteur" << endl <<flush;
            delete(msgRcv);
            
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = MESSAGE_ROBOT_STOP;
            rt_mutex_release(&mutex_move);
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                status = robot.Close();
            rt_mutex_release(&mutex_robot);
            cout << "Communication avec le robot " <<((status <0)?"non fermée" : "fermée") << endl <<flush;
            
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Close();
            rt_mutex_release(&mutex_monitor);
            
            rt_sem_v(&sem_closeCamera);
            statusCamera = false;
            
            robotStarted = 0;
            battery = BATTERY_UNKNOWN;
            arenaOK=0;
            getPosition=false;
            //exit(-1);
            
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }/*else if(msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            rt_sem_v(&sem_startBattery)    ;
        } */
        else if(msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            rt_sem_v(&sem_startCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            
            rt_sem_v(&sem_closeCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            
            rt_sem_v(&sem_findArenaCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            arenaOK=true;
            rt_sem_v(&sem_confirmFindArenaCamera)    ;
        }else if( msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            arenaOK=false;
            rt_sem_v(&sem_confirmFindArenaCamera)    ;
        }
        else if( msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            getPosition=true;
            
        }
        else if( msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            getPosition=false;
            
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}


/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
       // cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        //cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**
 * @brief Get battery level from robot and send it periodically to monitor.
 */
void Tasks::GetBatteryTask(void *arg) {
   
    MessageBattery * msgbat;
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
        
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    rt_task_set_periodic(NULL, TM_NOW, 500000000); 
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery update";
       
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            if(rs==1){
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgbat = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET)); 
                cout << msgbat->ToString()<<endl <<flush;
                rt_mutex_release(&mutex_robot);
                WriteInQueue(&q_messageToMon, msgbat);
            }        
    }
}

/**
* @brief Thread handling opening of the camera.
*/
void Tasks::OpenCamera(void *arg) {
    
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    Message *msgCam;
    while(1){
        rt_sem_p(&sem_startCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);

        cout << "Try opening camera" << endl << flush;
        if (camera.Open()){
            cout << "Camera opened successfully" << endl << flush;
            msgCam = new Message(MESSAGE_ANSWER_ACK);
            statusCamera=true;
            rt_sem_v(&sem_sendImageTask);
        }
        else {
            cout << "Failed to open camera" << endl << flush;
            msgCam = new Message(MESSAGE_ANSWER_NACK);
            statusCamera=false;
        }
        WriteInQueue(&q_messageToMon, msgCam);
        rt_mutex_release(&mutex_camera);
    }
}

/**
* @brief Thread handling closing of the camera.
*/
void Tasks::CloseCamera(void *arg) {
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    Message *msgCam;
    while(1){
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        
        cout << "Try closing camera" << endl << flush;
            
        camera.Close();
        cout << "Camera closed successfully" << endl << flush;
        statusCamera=false;
        rt_sem_v(&sem_stopSendImageTask);
        msgCam = new Message(MESSAGE_ANSWER_ACK);
        WriteInQueue(&q_messageToMon, msgCam); 
        
               
        rt_mutex_release(&mutex_camera);
            
     }
}

/**
* @brief Thread handling the send of image to the monitor.
*/
void Tasks::SendImageTask(void *arg) {
    MessageImg * msgSendImg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
   
    rt_task_set_periodic(NULL, TM_NOW, 100000000); 
  
    Img * image;
    Arena * arena;
    Position position;
    std::list<Position> listeRobot;
    MessagePosition * msgPosition;
    while (1) {
        rt_task_wait_period(NULL);
        cout << "attente de debut de tache " <<endl;
         rt_sem_p(&sem_sendImageTask, TM_INFINITE);
         cout << "debut de tache " <<endl;
         while(statusCamera){
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            try{
                image=new Img(camera.Grab());
                if(!arenaOK){
                    arena=new Arena(image->SearchArena());
                    if(!arena->IsEmpty()){
                        image->DrawArena(*arena);
                    }
                }else{
                     image->DrawArena(arenaOKByUser);
                     if(getPosition){
                         listeRobot = image->SearchRobot(arenaOKByUser);
                         if(!listeRobot.empty()){
                             image->DrawAllRobots(listeRobot);
                             msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, listeRobot.front());
                             
                         }else{
                             msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, position);
                             
                         }
                         
                             WriteInQueue(&q_messageToMon, msgPosition);
                     }
                }
                msgSendImg = new MessageImg(MESSAGE_CAM_IMAGE,image);
                WriteInQueue(&q_messageToMon, msgSendImg); 
            }
            catch(...){
                cout << "Erreur capture"<<endl;
            }
            rt_mutex_release(&mutex_camera);
        } 
    }
}

/**
* @brief Thread handling the search of the arena.
*/        
void Tasks::FindArenaCamera(void *arg) {
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    Message *msgArena;
    MessageImg * msgArenaImg;
    Img * image;
    Arena * arena;

    while(1){
        
        rt_sem_p(&sem_findArenaCamera, TM_INFINITE);
         rt_sem_v(&sem_closeCamera)    ;
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                try{
                    if (camera.Open()){
                        cout << "Camera opened successfully" << endl << flush;
                        image=new Img(camera.Grab());
                        arena=new Arena(image->SearchArena());
                    
                 if(!arena->IsEmpty()){
                    arenaOK=false;
                    image->DrawArena(*arena);
                   
                    msgArenaImg = new MessageImg(MESSAGE_CAM_IMAGE,image);
                    WriteInQueue(&q_messageToMon, msgArenaImg); 
                        
                        //================================
                    rt_sem_p(&sem_confirmFindArenaCamera, TM_INFINITE);
                    if(arenaOK == true){ //CONFIRM
                        cout << "Arena confirmed" << endl << flush;
                        arenaOKByUser = *arena;
                        msgArena = new Message(MESSAGE_ANSWER_ACK);
  

                    }else{//INFIRM
                        cout << "Arena infirmed" << endl << flush;
                        arenaOK=false;
                        msgArena = new Message(MESSAGE_ANSWER_ACK);
                    }

                  WriteInQueue(&q_messageToMon, msgArena); 
             
                  //=======================================
                 }else{
                    msgArena = new Message(MESSAGE_ANSWER_NACK);
           
        WriteInQueue(&q_messageToMon, msgArena);
                    
                 }
                  }else{
                     msgArena = new Message(MESSAGE_ANSWER_NACK);
           
        WriteInQueue(&q_messageToMon, msgArena);
                  }
                  }
         
                  catch(...){
                      cout << "Erreur capture"<<endl;
                        msgArena = new Message(MESSAGE_ANSWER_NACK);
           
        WriteInQueue(&q_messageToMon, msgArena);
                  }
        rt_sem_v(&sem_startCamera) ;
                  rt_mutex_release(&mutex_camera);
            
     }
}

/**
 * @brief Thread handling the loss of connexion with the robot.
 */
void Tasks::ConnexionRobotLost(void *arg){
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    int status ;
    int nbreEchec = 0;
    int rs;
    Message * answer;
    
    rt_task_set_periodic(NULL, TM_NOW, 2000000000); //2s period
    
    while(1){
        rt_task_wait_period(NULL); 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if(rs ==1){
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(new Message(MESSAGE_ROBOT_PING)); //envoie un ping au robot
            rt_mutex_release(&mutex_robot);
            
            if (!(answer->CompareID(MESSAGE_ANSWER_ACK))){
                nbreEchec++; //on incrémente le compteur d'erreur
                cout << "connection lost "<< nbreEchec << " times in a row"<<endl << flush;
            }
            else{
                nbreEchec = 0;
                cout << "connection good "<<endl << flush;
            }
            
            if(nbreEchec >= 3){
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                  status = robot.Close();
            rt_mutex_release(&mutex_robot);
            cout << "Communication avec le robot " <<((status <0)?"non fermée" : "fermée") << endl <<flush;
            
            }
            
        }
        
    }
    
}