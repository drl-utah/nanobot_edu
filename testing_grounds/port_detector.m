clc 
clear all

startlist = serialportlist("available")
input("Connect your Arduino via USB cable and then press enter\n");
endlist = serialportlist("available");

%%
for int i=0:1:length(startlist)
    
end

