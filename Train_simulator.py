import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random
import math




###############################################################################

# ONLY CHANGE THESE INPUTS FOR THE CODE TO WORK PROPERLY

# Initialize input values
trials=10
#incl_angle=np.pi/6*1 # Keep the angle between 0 and +pi/6 radians
incl_angle = 0;
g=10
mass_cart=100 # [kg]

# Tune the constants
K_p=300
K_d=300
K_i=0
###############################################################################


# Generate random x-positions for a falling cube
def set_x_ref(incl_angle):
    #rand_h=60
    rand_h=random.uniform(0,120)
    rand_v=random.uniform(20+120*np.tan(incl_angle)+6.5,40+120*np.tan(incl_angle)+6.5)
    return rand_h,rand_v


dt=0.01
t0=0
t_end=5
frame_amount=int(t_end/dt)*trials
t=np.arange(t0,t_end+dt,dt)

F_g=mass_cart*g

displ_rail=np.zeros((trials,len(t)))
v_rail=np.zeros((trials,len(t)))
a_rail=np.zeros((trials,len(t)))
pos_x_h=np.zeros((trials,len(t)))
pos_y_h=np.zeros((trials,len(t)))
v_x=np.zeros((trials,len(t)))
v_y=np.zeros((trials,len(t)))
e=np.zeros((trials,len(t)))
e_dot=np.zeros((trials,len(t)))
e_int=np.zeros((trials,len(t)))

pos_x_v=np.zeros((trials,len(t)))
pos_y_v=np.zeros((trials,len(t)))
F_ga_x=-F_g*np.sin(incl_angle)*np.cos(incl_angle)
F_ga_y=-F_g*np.sin(incl_angle)**2
F_a=np.zeros((trials,len(t)))
F_a_x=np.zeros((trials,len(t)))
F_a_y=np.zeros((trials,len(t)))
F_net_x=np.ones((trials,len(t)))*F_ga_x
F_net_y=np.ones((trials,len(t)))*F_ga_y

t_cube=t
init_pos_x=120
init_pos_y=(120*np.tan(incl_angle)+6.5)
init_vel_x=0
init_vel_y=0

bbox_props_success=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='g',lw='1')
bbox_props_again=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='r',lw='1')

trials_magn=trials
history=np.ones(trials)
min_v=0
max_v=0
min_a=0
max_a=0
min_e=0
max_e=0
min_e_dot=0
max_e_dot=0
min_e_int=0
max_e_int=0
min_F_a=0
max_F_a=0

while(trials>0): # Determines how many times cube falls down
    pos_x_v_ref=set_x_ref(incl_angle)[0]
    init_alt=set_x_ref(incl_angle)[1]
    times=trials_magn-trials
    pos_x_v[times]=pos_x_v_ref
    pos_y_v[times]=init_alt-g/2*(t_cube)**2
    win=False
    delta=1

    # Implement PID for pos_x_h
    for i in range(1,len(t)):
        if i==1:
            pos_x_h[times][0]=init_pos_x
            pos_y_h[times][0]=init_pos_y
            v_x[times][0]=init_vel_x
            v_y[times][0]=init_vel_y
        e[times][i-1]=pos_x_v_ref-pos_x_h[times][i-1]
        if pos_y_v[times][i-1]<10 or pos_y_v[times][i-1]>-10:
            e[times][i-1]=e[times][i-1]
        else:
            e[times][i-1]=e[times][i-1]*(abs(e[times][i-1])/abs(pos_y_v[times][i-1]))
        if i>1:
            e_dot[times][i-1]=(e[times][i-1]-e[times][i-2])/dt
            e_int[times][i-1]=e_int[times][i-2]+(e[times][i-2]+e[times][i-1])/2*dt
        if i==len(t)-1:
            e[times][-1]=e[times][-2]
            e_dot[times][-1]=e_dot[times][-2]
            e_int[times][-1]=e_int[times][-2]
        F_a[times][i]=K_p*e[times][i-1]+K_d*e_dot[times][i-1]+K_i*e_int[times][i-1]
        F_a_x[times][i]=F_a[times][i]*np.cos(incl_angle)
        F_a_y[times][i]=F_a[times][i]*np.sin(incl_angle)
        F_net_x[times][i]=F_a_x[times][i]+F_ga_x
        F_net_y[times][i]=F_a_y[times][i]+F_ga_y
        if F_net_x[times][i]<0 or F_net_y[times][i]<0:
            a_rail[times][i]=-1/mass_cart*(F_net_x[times][i]**2+F_net_y[times][i]**2)**(0.5)
        else:
            a_rail[times][i]=1/mass_cart*(F_net_x[times][i]**2+F_net_y[times][i]**2)**(0.5)

        # Transition to a new time instance
        v_x[times][i]=v_x[times][i-1]+1/mass_cart*(F_net_x[times][i-1]+F_net_x[times][i])/2*dt
        v_y[times][i]=v_y[times][i-1]+1/mass_cart*(F_net_y[times][i-1]+F_net_y[times][i])/2*dt
        pos_x_h[times][i]=pos_x_h[times][i-1]+(v_x[times][i-1]+v_x[times][i])/2*dt
        pos_y_h[times][i]=pos_y_h[times][i-1]+(v_y[times][i-1]+v_y[times][i])/2*dt
        if v_x[times][i]<0 or v_y[times][i]<0:
            v_rail[times][i]=-(v_x[times][i]**2+v_y[times][i]**2)**(0.5)
        else:
            v_rail[times][i]=(v_x[times][i]**2+v_y[times][i]**2)**(0.5)

        if pos_x_h[times][i]<0 or pos_y_h[times][i]<0:
            displ_rail[times][i]=-(pos_x_h[times][i]**2+(pos_y_h[times][i]-6.5)**2)**(0.5)
        else:
            displ_rail[times][i]=(pos_x_h[times][i]**2+(pos_y_h[times][i]-6.5)**2)**(0.5)


        # Try to catch it
        if (pos_x_h[times][i]-5<pos_x_v[times][0]+3 and pos_x_h[times][i]+5>pos_x_v[times][1]-3) or win==True:
            if (pos_y_h[times][i]+3<pos_y_v[times][i]-2 and pos_y_h[times][i]+8>pos_y_v[times][i]+2) or win==True:
                win=True
                if delta==1:
                    change=pos_x_h[times][i]-pos_x_v[times][i]
                    delta=0
                pos_x_v[times][i]=pos_x_h[times][i]-change
                pos_y_v[times][i]=pos_y_h[times][i]+5

    init_pos_x=pos_x_h[times][-1]+v_x[times][-1]*dt
    init_pos_y=pos_y_h[times][-1]+v_y[times][-1]*dt
    init_vel_x=v_x[times][-1]
    init_vel_y=v_y[times][-1]
    history[times]=delta
    if min(v_rail[times]) < min_v:
        min_v=min(v_rail[times])
    if max(v_rail[times]) > max_v:
        max_v=max(v_rail[times])
    if min(a_rail[times]) < min_a:
        min_a=min(a_rail[times])
    if max(a_rail[times]) > max_a:
        max_a=max(a_rail[times])
    if min(e[times]) < min_e:
        min_e=min(e[times])
    if max(e[times]) > max_e:
        max_e=max(e[times])
    if min(e_dot[times]) < min_e_dot:
        min_e_dot=min(e_dot[times])
    if max(e_dot[times]) > max_e_dot:
        max_e_dot=max(e_dot[times])
    if min(e_int[times]) < min_e_int:
        min_e_int=min(e_int[times])
    if max(e_int[times]) > max_e_int:
        max_e_int=max(e_int[times])
    if min(F_a[times]) < min_F_a:
        min_F_a=min(F_a[times])
    if max(F_a[times]) > max_F_a:
        max_F_a=max(F_a[times])
    trials=trials-1


len_t=len(t)-1
def update_plot(num):

    platform.set_data([pos_x_h[int(num/len_t)][num-int(num/len_t)*len_t]-3.1,
    pos_x_h[int(num/len_t)][num-int(num/len_t)*len_t]+3.1],
    [pos_y_h[int(num/len_t)][num-int(num/len_t)*len_t],
    pos_y_h[int(num/len_t)][num-int(num/len_t)*len_t]])

    cube.set_data([pos_x_v[int(num/len_t)][num-int(num/len_t)*len_t]-1,pos_x_v[int(num/len_t)][num-int(num/len_t)*len_t]+1],
    [pos_y_v[int(num/len_t)][num-int(num/len_t)*len_t],pos_y_v[int(num/len_t)][num-int(num/len_t)*len_t]])

    if trials_magn*len_t==num+1 and num>0: # All attempts must be successful
        if sum(history)==0:
            success.set_text('Nice Tuning!')
        else:
            again.set_text('Bad Tuning Try Other Values!')

    displ_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        displ_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    v_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        v_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    a_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        a_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_dot_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e_dot[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_int_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e_int[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    # F_a_f.set_data(t[0:(num-int(num/len_t)*len_t)],
    #     F_a[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    return platform,cube,success,again,displ_rail_f,v_rail_f,a_rail_f,\
        e_f,e_dot_f,e_int_f,#F_a_f


fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
gs=gridspec.GridSpec(4,3)

# Create game window
ax_main=fig.add_subplot(gs[0:3,0:2],facecolor=(0.9,0.9,0.9))
rail=ax_main.plot([0,120],[5,120*np.tan(incl_angle)+5],'k',linewidth=6)
platform,=ax_main.plot([],[],'b',linewidth=18)
cube,=ax_main.plot([],[],'k',linewidth=14)

plt.xlim(0,120)
plt.ylim(0,120)
plt.xticks(np.arange(0,121,10))
plt.yticks(np.arange(0,121,10))
plt.grid(True)

success=ax_main.text(40,60,'',size='20',color='g',bbox=bbox_props_success)
again=ax_main.text(30,60,'',size='20',color='r',bbox=bbox_props_again)
copyright=ax_main.text(0,122,'© 2021 CROCOMARINES',size=12)



# Plot windows
ax1v=fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
displ_rail_f,=ax1v.plot([],[],'-b',linewidth=2,label='displ. on rails [m]')
plt.xlim(t0,t_end)
plt.ylim(-20,140)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax2v=fig.add_subplot(gs[1,2],facecolor=(0.9,0.9,0.9))
v_rail_f,=ax2v.plot([],[],'-b',linewidth=2,label='velocity on rails [m/s]')
plt.xlim(t0,t_end)
plt.ylim(min_v-10,max_v+10)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax3v=fig.add_subplot(gs[2,2],facecolor=(0.9,0.9,0.9))
a_rail_f,=ax3v.plot([],[],'-b',linewidth=2,label='accel. on rails [m/s^2] = F_net/m_platf.')
plt.xlim(t0,t_end)
plt.ylim(min_a-10,max_a+10)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

# ax3v_F=ax3v.twinx()
# F_a_f,=ax3v_F.plot([],[],'-r',linewidth=2,label='Force applied = F_net - F_grav. [N]')
# plt.xlim(t0,t_end)
# plt.ylim(min_F_a-10,max_F_a+10)
# plt.grid(True)
# plt.legend(loc='upper right',fontsize='small')

ax1h=fig.add_subplot(gs[3,0],facecolor=(0.9,0.9,0.9))
e_f,=ax1h.plot([],[],'-b',linewidth=2,label='horizontal error [m]')
plt.xlim(t0,t_end)
plt.ylim(min_e-10,max_e+10)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax2h=fig.add_subplot(gs[3,1],facecolor=(0.9,0.9,0.9))
e_dot_f,=ax2h.plot([],[],'-b',linewidth=2,label='change of horiz. error [m/s]')
plt.xlim(t0,t_end)
plt.ylim(min_e_dot-10,max_e_dot+10)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax3h=fig.add_subplot(gs[3,2],facecolor=(0.9,0.9,0.9))
e_int_f,=ax3h.plot([],[],'-b',linewidth=2,label='sum of horiz. error [m*s]')
plt.xlim(t0,t_end)
plt.ylim(min_e_int-10,max_e_int+10)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

pid_ani=animation.FuncAnimation(fig,update_plot,
    frames=frame_amount,interval=20,repeat=False,blit=True)

plt.show()
