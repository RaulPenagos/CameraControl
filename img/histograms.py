import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import ScalarFormatter
import scipy.stats as ss
import numpy as np

# Plots with Tex. Must install:  (base, recommended, full)
# sudo apt install texlive-latex-full
mpl.rcParams['text.usetex'] = True
mpl.rcParams['font.family'] = 'serif'

# mpl.rcParams['mathtext.fontset'] = 'custom'
# mpl.rcParams['mathtext.rm'] = 'Times New Roman'
# mpl.rcParams['mathtext.it'] = 'Times New Roman:italic'
# mpl.rcParams['mathtext.bf'] = 'Times New Roman:bold'


# mpl.rcParams.update(mpl.rcParamsDefault)



def binchart(data, varname:str = 'x', filename: str = 'default', xunits: str = 'cm'):
    """
    Makes binchart
    Input:
        data    data to plot
        varname name for the axis and file. Tex compatible: r'...'
    """
    # data = data /max(data) *100

    plt.figure(figsize=(6, 5))
    plt.grid(linestyle='dotted')
    ax = plt.gca()
    plt.hist(data, bins = 30, color = 'lightseagreen', label = r'$\delta$ ' + varname)
    plt.legend()
    plt.title(r'$\Delta$' + varname)
    plt.xlabel(r'$\delta $' + varname + f'/ {xunits}', fontsize = 15)
    plt.ylabel('Frec.', fontsize = 15)
    plt.savefig(f'./img/graph_{filename}.png')
    # plt.show()


def binchart_fit(data, varname:str = 'x', filename: str = 'default', xunits: str = 'cm'):
    """
        Makes binchart and normal distribution fit
    """
    normfit = ss.norm.fit(data)

    mu = normfit[0]
    sigma = normfit[1]
    lim = 1.1 * max(abs(mu - min(data)), abs(mu - max(data)))

    datasort = np.linspace(mu - lim, mu + lim,  100)


    plt.figure(figsize=(6, 5))
    plt.grid(linestyle='dotted')
    ax = plt.gca()
    plt.hist(data, bins = 30, density = True, color = 'lightseagreen', label = r'$\delta$ ' + varname)
    plt.plot(datasort, ss.norm.pdf(datasort, *normfit), color = 'r', label = 'Normal Fit')
    plt.axvline(0, linestyle = 'dotted', color = 'k')
    plt.xlim(mu-lim, mu+lim)
    plt.legend(fontsize = 12)

    # Forzar notación científica en eje Y
    ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))  # Forza siempre notación científica

    # Usar ScalarFormatter con MathText para que el offset (10^x) se vea bonito
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_powerlimits((0, 0))  # También forza notación científica
    formatter.set_scientific(True)

    ax.yaxis.set_major_formatter(formatter)
    ax.yaxis.offsetText.set_fontsize(15)  # Tamaño de fuente del "×10⁻⁵"
    # ax.yaxis.offsetText.set_x()  # Opcional: mueve un poco el offset si lo necesitas
    

    # plt.title(r'$pdf ($' + varname + r'$)$ \n', fontsize = 20)
    plt.xlabel(r'$\delta $' + varname + f'/ {xunits}', fontsize = 20)
    plt.ylabel('Frec.', fontsize = 20)
    plt.text(0.20, 0.8, r'$\mu:$' + f'{mu:.2e} {xunits}',  horizontalalignment='center', fontsize = 18, verticalalignment='center', transform=ax.transAxes)
    plt.text(0.20, 0.7, r'$\sigma:$'+ f'{sigma:.2e} {xunits}',  horizontalalignment='center', fontsize = 18, verticalalignment='center', transform=ax.transAxes)

    plt.savefig(f'./img/graph_{filename}.png')
    # plt.show()

