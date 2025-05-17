

import logging 

global sptime, spvalu, pptime, ppvalu, mltime, mlvalu, mrtime, mrvalu, mftime, mfvalu, altime, alvalu
global sptime2, spvalu2, pptime2, ppvalu2, mltime2, mlvalu2, mrtime2, mrvalu2, mftime2, mfvalu2, altime2, alvalu2


# ---------- Read recipe from file

def read_recipe(filename, timename, valuname) :
    xxtime = []
    xxvalu = []

    logging.info("Reading recipe from file: " + filename)

    try :
        
        fin = open(filename, "r")
        
        tt = []
        vv = []
        
        for line in fin:
            
            # Skip comment lines and short/empty lines
            if len(line)<3 : continue
            if line.lstrip()[0] == '#' : continue 

            # Find line with: sptime ...  [ ... ]
            p0 = line.find(timename)
            if p0>=0 :
                p1 = line.find('[')
                p2 = line.find(']')
                if p1>p0 and p2>p1 :
                    tt = line[p1+1:p2].split(',')
                    continue

            # Find line with: spvalue ...  [ ... ]
            p0 = line.find(valuname)
            if p0>=0 :
                p1 = line.find('[')
                p2 = line.find(']')
                if p1>p0 and p2>p1 :
                    vv = line[p1+1:p2].split(',')
                    continue

        if len(tt)>0 or len(vv)>0:
            if len(tt) == len(vv) :
                for i in range(len(tt)) :
                    try :
                        tti = float(tt[i])
                        vvi = float(vv[i])
                    except :
                        logging.error('Wrong number format: '+tt[i]+' / '+vv[i])
                    else :
                        xxtime.append(tti)
                        xxvalu.append(vvi)
            else :
                logging.error('Different array length '+str(len(tt))+' / '+str(len(vv)))
                
    except :
        logging.error('File not found!')
                        
    logging.info(timename + ' = ')
    logging.info(xxtime)
    logging.info(valuname + ' = ')
    logging.info(xxvalu)
    logging.info('---') 

    return xxtime, xxvalu

# ---------- Add time offset and make arrays for plots
# Ploting arrays with step-like profiles and adjusted scaling 


def set_sp(t0) :
    global sptime, spvalu, sptime2, spvalu2
    tx = 1.0
    sptime, spvalu = read_recipe('recipes.txt', 'sptime', 'spvalu') 
    sptime2 = []
    spvalu2 = []
    for i in range(len(sptime)):
        sptime[i] = sptime[i]*tx+t0
        sptime2.append(sptime[i])
        spvalu2.append(spvalu[i])

def set_pp(t0) :
    global pptime, ppvalu, pptime2, ppvalu2
    tx = 1.0
    pptime, ppvalu = read_recipe('recipes.txt', 'pptime', 'ppvalu') 
    pptime2 = []
    ppvalu2 = []
    for i in range(len(pptime)):
        pptime[i] = pptime[i]*tx+t0
        pptime2.append(pptime[i])
        ppvalu2.append(ppvalu[i])

def set_ml(t0) :
    global mltime, mlvalu, mltime2, mlvalu2
    tx = 1.0
    mltime, mlvalu = read_recipe('recipes.txt', 'mltime', 'mlvalu')  
    mltime2 = []
    mlvalu2 = []
    for i in range(len(mltime)):
        mltime[i] = mltime[i]*tx+t0
        if mlvalu[i-1]!=mlvalu[i]:
            mltime2.append(mltime[i]-1)
            mlvalu2.append(mlvalu[i-1])
        mltime2.append(mltime[i])
        mlvalu2.append(mlvalu[i])

def set_mr(t0) :
    global mrtime, mrvalu, mrtime2, mrvalu2
    tx = 1.0
    mrtime, mrvalu = read_recipe('recipes.txt', 'mrtime', 'mrvalu')  
    mrtime2 = []
    mrvalu2 = []
    for i in range(len(mrtime)):
        mrtime[i] = mrtime[i]*tx+t0
        if mrvalu[i-1]!=mrvalu[i]:
            mrtime2.append(mrtime[i]-1)
            mrvalu2.append(mrvalu[i-1])
        mrtime2.append(mrtime[i])
        mrvalu2.append(mrvalu[i])

def set_mf(t0) :
    global mftime, mfvalu, mftime2, mfvalu2
    tx = 1.0  
    mftime, mfvalu = read_recipe('recipes.txt', 'mftime', 'mfvalu')    
    mftime2 = []
    mfvalu2 = []
    for i in range(len(mftime)):
        mftime[i] = mftime[i]*tx+t0
        if mfvalu[i-1]!=mfvalu[i]:
            mftime2.append(mftime[i]-1)
            mfvalu2.append(mfvalu[i-1])
        mftime2.append(mftime[i])
        mfvalu2.append(mfvalu[i])

def set_al(t0) :
    global altime, alvalu, altime2, alvalu2
    tx = 1.0
    altime, alvalu = read_recipe('recipes.txt', 'altime', 'alvalu')  
    altime2 = []
    alvalu2 = []
    for i in range(len(altime)):
        altime[i] = altime[i]*tx+t0
        if alvalu[i]>0:
            altime2.append(altime[i])
            alvalu2.append(alvalu[i])


# ---------- Interpolation function


def interpol(at, av, tt2, opt) :
    
    asize = len(at)
    
    if asize == 0 :
        return 0

    # find next time point greater than/equal to tt
    i = 1 
    while i < asize :
        if at[i] < tt2 : i += 1
        else: break
    
    # single value in array
    if asize < 2: return av[0]  
    
    # constant extrapolation
    if tt2 > at[asize-1] : return av[asize-1] 
    if tt2 < at[0] : return av[0];
    
    # switch to next value if time >= tt2  
    if opt == 0 :
        if tt2 < at[i] : return av[i-1]   
        else : return av[i]; 
    
    # linear interpolation
    if opt == 1 :  
        adt = at[i] - at[i-1]
        if adt == 0 : return av[i-1]  # zero time interval
        return av[i-1] + (av[i]-av[i-1])*(tt2-at[i-1])/adt

  



