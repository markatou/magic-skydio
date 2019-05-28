package lggltl.cleanup;

import lggltl.cleanup.state.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ngopalan on 11/30/17.
 */
public class CreateStartStates {

    /*
    Colours that work red, green, blue, yellow, cyan,
    pink, gray, white,
     */
    public CleanupState getState(String whichState){
        CleanupState cs;
        switch (whichState.split("_")[0]){
            case "2":
                cs = twoCorridorState(whichState);
                break;
            case "3":
                cs = threeCorridorState(whichState);
                break;
            case "4":
                cs = twoCorridorStateFlipped(whichState);
                break;
            case "5":
                cs = longCorridorState(whichState);
                break;
            default:
                cs = null;
                System.err.println("incorrect state specification");
                break;
        }

        return cs;
    }

    private CleanupState twoCorridorState(String whichState) {
        String[] colors = whichState.split("_");


        int y1 = 4;
        int y2 = 8;
        int y3 = 12;


        int x1 = 4;
        int x2 = 8;

        int dx0 = 2;
        int dx1 = 6;

        CleanupRoom r1 = new CleanupRoom(CleanupDomain.CLASS_ROOM+0,y1, 0, 0, x2, colors[1]);
        CleanupRoom r2 = new CleanupRoom(CleanupDomain.CLASS_ROOM+1, y2, 0, y1, x1, colors[2]);
        CleanupRoom r3 = new CleanupRoom(CleanupDomain.CLASS_ROOM+2, y2, x1, y1, x2, colors[3]);
        CleanupRoom r4 = new CleanupRoom(CleanupDomain.CLASS_ROOM+3, y3, 0, y2, x2, colors[4]);
        List<CleanupRoom> rooms = new ArrayList<CleanupRoom>();
        rooms.add(r1);
        rooms.add(r2);
        rooms.add(r3);
        rooms.add(r4);

        CleanupDoor d1 = new CleanupDoor(CleanupDomain.CLASS_DOOR+0,0,y1, dx0, y1, dx0,false);
        CleanupDoor d2 = new CleanupDoor(CleanupDomain.CLASS_DOOR+1,0,y1, dx1, y1, dx1,false);
        CleanupDoor d3 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,y2, dx0, y2, dx0,false);
        CleanupDoor d4 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,y2, dx1, y2, dx1,false);
        List<CleanupDoor> doors = new ArrayList<CleanupDoor>();
        doors.add(d1);
        doors.add(d2);
        doors.add(d3);
        doors.add(d4);

        CleanupAgent agent = new CleanupAgent(CleanupDomain.CLASS_AGENT+0, 2, 2);
        agent.directional = true;
        agent.currentDirection = "south";


//        CleanupBlock block1 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+0,6, 6,"chair", "yellow");
//        CleanupBlock block2 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+1,6,10,"basket", "red");
//        CleanupBlock block3 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+2,2,10,"bag", "magenta");

        List<CleanupBlock> blocks = new ArrayList<CleanupBlock>();
//        blocks.add(block1);

//        if(numObjects>=2)        blocks.add(block2);
//        if(numObjects>=3)        blocks.add(block3);


        CleanupState s = new CleanupState(agent,blocks, doors, rooms);
        return s;

    }

    private CleanupState longCorridorState(String whichState) {
        String[] colors = whichState.split("_");


        int y1 = 4;
        int y2 = 8;
        int y3 = 12;
        int y4 = 16;
        int y5 = 20;


        int x1 = 4;
        int x2 = 8;

        int dx0 = 2;
        int dx1 = 6;

        CleanupRoom r1 = new CleanupRoom(CleanupDomain.CLASS_ROOM+0,y1, 0, 0, x2, colors[1]);
        CleanupRoom r2 = new CleanupRoom(CleanupDomain.CLASS_ROOM+1, y2, 0, y1, x1, colors[2]);
        CleanupRoom r3 = new CleanupRoom(CleanupDomain.CLASS_ROOM+2, y2, x1, y1, x2, colors[3]);
        CleanupRoom r4 = new CleanupRoom(CleanupDomain.CLASS_ROOM+3, y3, 0, y2, x2, colors[1]);
        CleanupRoom r5 = new CleanupRoom(CleanupDomain.CLASS_ROOM+4, y4, 0, y3, x1, colors[4]);
        CleanupRoom r6 = new CleanupRoom(CleanupDomain.CLASS_ROOM+5, y4, x1, y3, x2, colors[5]);
        CleanupRoom r7 = new CleanupRoom(CleanupDomain.CLASS_ROOM+6, y5, 0, y4, x2, colors[6]);
        List<CleanupRoom> rooms = new ArrayList<CleanupRoom>();
        rooms.add(r1);
        rooms.add(r2);
        rooms.add(r3);
        rooms.add(r4);
        rooms.add(r5);
        rooms.add(r6);
        rooms.add(r7);


        CleanupDoor d1 = new CleanupDoor(CleanupDomain.CLASS_DOOR+0,0,y1, dx0, y1, dx0,false);
        CleanupDoor d2 = new CleanupDoor(CleanupDomain.CLASS_DOOR+1,0,y1, dx1, y1, dx1,false);
        CleanupDoor d3 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,y2, dx0, y2, dx0,false);
        CleanupDoor d4 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,y2, dx1, y2, dx1,false);
        CleanupDoor d5 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,y3, dx0, y3, dx0,false);
        CleanupDoor d6 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,y3, dx1, y3, dx1,false);
        CleanupDoor d7 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,y4, dx0, y4, dx0,false);
        CleanupDoor d8 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,y4, dx1, y4, dx1,false);
        List<CleanupDoor> doors = new ArrayList<CleanupDoor>();
        doors.add(d1);
        doors.add(d2);
        doors.add(d3);
        doors.add(d4);
        doors.add(d5);
        doors.add(d6);
        doors.add(d7);
        doors.add(d8);

        CleanupAgent agent = new CleanupAgent(CleanupDomain.CLASS_AGENT+0, 2, 2);
        agent.directional = true;
        agent.currentDirection = "south";


//        CleanupBlock block1 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+0,5, 4,"chair", "blue");
//        CleanupBlock block2 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+1,6,10,"basket", "red");
//        CleanupBlock block3 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+2,2,10,"bag", "magenta");

        List<CleanupBlock> blocks = new ArrayList<CleanupBlock>();
//        blocks.add(block1);

//        if(numObjects>=2)        blocks.add(block2);
//        if(numObjects>=3)        blocks.add(block3);


        CleanupState s = new CleanupState(agent,blocks, doors, rooms);
        return s;

    }

    private CleanupState twoCorridorStateFlipped(String whichState) {
        String[] colors = whichState.split("_");


//        int x1 = 4;
//        int x2 = 8;
//        int x3 = 12;
//
//
//        int y1 = 4;
//        int y2 = 8;
//
//        int dy0 = 2;
//        int dy1 = 6;
//
//        CleanupRoom r1 = new CleanupRoom(CleanupDomain.CLASS_ROOM+0,x2, 0, 0, y1, colors[1]);
//        CleanupRoom r2 = new CleanupRoom(CleanupDomain.CLASS_ROOM+1, y2, 0, y1, x1, colors[2]);
//        CleanupRoom r3 = new CleanupRoom(CleanupDomain.CLASS_ROOM+2, y2, x1, y1, x2, colors[3]);
//        CleanupRoom r4 = new CleanupRoom(CleanupDomain.CLASS_ROOM+3, y3, 0, y2, x2, colors[4]);


        int y1 = 4;
        int y2 = 8;
        int y3 = 12;


        int x1 = 4;
        int x2 = 8;

        int dx0 = 2;
        int dx1 = 6;

        CleanupRoom r1 = CleanupRoomFlipped(CleanupDomain.CLASS_ROOM+0,y1, 0, 0, x2, colors[1]);
        CleanupRoom r2 =  CleanupRoomFlipped(CleanupDomain.CLASS_ROOM+1, y2, 0, y1, x1, colors[2]);
        CleanupRoom r3 =  CleanupRoomFlipped(CleanupDomain.CLASS_ROOM+2, y2, x1, y1, x2, colors[3]);
        CleanupRoom r4 =  CleanupRoomFlipped(CleanupDomain.CLASS_ROOM+3, y3, 0, y2, x2, colors[4]);

        List<CleanupRoom> rooms = new ArrayList<CleanupRoom>();
        rooms.add(r1);
        rooms.add(r2);
        rooms.add(r3);
        rooms.add(r4);

        CleanupDoor d1 = new CleanupDoor(CleanupDomain.CLASS_DOOR+0,0,dx0,y1, dx0, y1, false);
        CleanupDoor d2 = new CleanupDoor(CleanupDomain.CLASS_DOOR+1,0,dx1, y1, dx1, y1,false);
        CleanupDoor d3 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,dx0, y2,dx0, y2,false);
        CleanupDoor d4 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,dx1, y2,dx1, y2,false);
        List<CleanupDoor> doors = new ArrayList<CleanupDoor>();
        doors.add(d1);
        doors.add(d2);
        doors.add(d3);
        doors.add(d4);

        CleanupAgent agent = new CleanupAgent(CleanupDomain.CLASS_AGENT+0, 1, 1);
        agent.directional = true;
        agent.currentDirection = "south";


        CleanupBlock block1 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+0,2, 2,"fire", "yellow");
//        CleanupBlock block2 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+1,6,10,"basket", "red");
//        CleanupBlock block3 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+2,2,10,"bag", "magenta");

        List<CleanupBlock> blocks = new ArrayList<CleanupBlock>();
        blocks.add(block1);

//        if(numObjects>=2)        blocks.add(block2);
//        if(numObjects>=3)        blocks.add(block3);


        CleanupState s = new CleanupState(agent,blocks, doors, rooms);
        return s;

    }


    public CleanupRoom CleanupRoomFlipped(String name, int top, int left, int bottom, int right, String colour){
        return new CleanupRoom(name,right, bottom, left, top, colour);
    }


    private CleanupState threeCorridorState(String whichState) {


        String[] colors = whichState.split("_");


        int y1 = 4;
        int y2 = 8;
        int y3 = 12;


        int x1 = 3;
        int x2 = 7;

        CleanupRoom r1 = new CleanupRoom(CleanupDomain.CLASS_ROOM+0,y1, 0, 0, x2, colors[1]);
        CleanupRoom r2 = new CleanupRoom(CleanupDomain.CLASS_ROOM+1, y2, 0, y1, x1, colors[2]);
        CleanupRoom r3 = new CleanupRoom(CleanupDomain.CLASS_ROOM+2, y2, x1, y1, x2, colors[3]);
        CleanupRoom r4 = new CleanupRoom(CleanupDomain.CLASS_ROOM+3, y3, 0, y2, x2, colors[4]);
        List<CleanupRoom> rooms = new ArrayList<CleanupRoom>();
        rooms.add(r1);
        rooms.add(r2);
        rooms.add(r3);
        rooms.add(r4);

        CleanupDoor d1 = new CleanupDoor(CleanupDomain.CLASS_DOOR+0,0,y1, 1, y1, 1,false);
        CleanupDoor d2 = new CleanupDoor(CleanupDomain.CLASS_DOOR+1,0,y1, 5, y1, 5,false);
        CleanupDoor d3 = new CleanupDoor(CleanupDomain.CLASS_DOOR+2,0,y2, 1, y2, 1,false);
        CleanupDoor d4 = new CleanupDoor(CleanupDomain.CLASS_DOOR+3,0,y2, 5, y2, 5,false);
        List<CleanupDoor> doors = new ArrayList<CleanupDoor>();
        doors.add(d1);
        doors.add(d2);
        doors.add(d3);
        doors.add(d4);

        CleanupAgent agent = new CleanupAgent(CleanupDomain.CLASS_AGENT+0, 2, 2);
        agent.directional = true;
        agent.currentDirection = "south";


//        CleanupBlock block1 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+0,5, 4,"chair", "blue");
//        CleanupBlock block2 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+1,6,10,"basket", "red");
//        CleanupBlock block3 = new CleanupBlock(CleanupDomain.CLASS_BLOCK+2,2,10,"bag", "magenta");

        List<CleanupBlock> blocks = new ArrayList<CleanupBlock>();
//        blocks.add(block1);

//        if(numObjects>=2)        blocks.add(block2);
//        if(numObjects>=3)        blocks.add(block3);


        CleanupState s = new CleanupState(agent,blocks, doors, rooms);
        return s;
    }
}
