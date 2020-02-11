from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
import plotly.graph_objs as go
from plotly import subplots

def createScatters(trace1, trace2):
    trace1X = go.Scatter(
        x=trace1.index,
        y=trace1.x,
        mode='lines',
        line = dict(color='Red')
    )

    trace1Y = go.Scatter(
        x=trace1.index,
        y=trace1.y,
        mode='lines',
        line = dict(color='Red')
    )
    
    trace1Z = go.Scatter(
        x=trace1.index,
        y=trace1.z,
        mode='lines',
        line = dict(color='Red')
    )
    
    trace2X = go.Scatter(
        x=trace2.index,
        y=trace2.x,
        mode='markers',
        marker=dict(color='Green')
    )

    trace2Y = go.Scatter(
        x=trace2.index,
        y=trace2.y,
        mode='markers',
        marker=dict(color='Green')
    )
    
    trace2Z = go.Scatter(
        x=trace2.index,
        y=trace2.z,
        mode='markers',
        marker=dict(color='Green')
    )
    
    return ([trace1X,trace1Y,trace1Z],[trace2X,trace2Y,trace2Z])
    
def addScatters(figure,allScatters,keptScatters):
    for i,scatterSet in enumerate(allScatters):
        for j,scatter in enumerate(scatterSet):
            figure.append_trace(scatter,i+1,j+1)
            
    for i,scatterSet in enumerate(keptScatters):
        for j,scatter in enumerate(scatterSet):
            figure.append_trace(scatter,i+1,j+1)
            
def plot2DSub(filename,title,proximalAll,proximalKept,distalAll,distalKept,orientationAll,orientationKept,height=1200):
    
    (spall,spkept)=createScatters(proximalAll,proximalKept)
    (sdall,sdkept)=createScatters(distalAll,distalKept)
    (soall,sokept)=createScatters(orientationAll,orientationKept)
    
    figure = subplots.make_subplots(rows=3,cols=3,print_grid=False,subplot_titles=('X','Y','Z','X','Y','Z','X','Y','Z'));
    
    addScatters(figure,[spall,sdall,soall],[spkept,sdkept,sokept])
    
    for i in range(9):
        figure['layout']['xaxis' + str(i+1)].update(title='Index')
        
    for i in range(3):
        for j in range(3):
            if i==2:
                figure['layout']['yaxis' + str(i*3+j+1)].update(title='Orientation (rad)')
            else:
                figure['layout']['yaxis' + str(i*3+j+1)].update(title='Position (m)')

    figure['layout'].update(title=title)
    figure['layout'].update(height=height)
    figure['layout'].update(showlegend=False)
    
    plot(figure, filename=filename)