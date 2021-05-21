package frc.robot.Autonomous;

import java.util.Set;
import java.util.TreeMap;

import frc.robot.Autonomous.Modes.AutoMode;

public class AutoModeList {

    private TreeMap<String, AutoMode> modeList = new TreeMap<String, AutoMode>();

    public void add(AutoMode in){
        modeList.put(in.humanReadableName, in);
    }

    public AutoMode get(String name){
        return modeList.get(name);
    }

    public String[] getNameList(){
        Set<String> keySet = modeList.keySet();
        return keySet.toArray(new String[keySet.size()]);
    }

    public AutoMode getDefault(){
        return modeList.firstEntry().getValue(); //TBD - just the first thing added?
    }
    
}
