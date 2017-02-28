#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals
import rospy
import rospkg
import sys
import os
import json
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from collections import OrderedDict

if __name__ == "__main__":
    results_path=rospkg.RosPack().get_path("ogm_server")+'/scripts/benchmarking_results/evaluation/map1/CUSTOMS/results.json'
    with open(results_path, 'r') as p:
        json_list = json.load(p, object_pairs_hook=OrderedDict)
        df = pd.DataFrame(json_list, columns= ['gaussianBlur2', 'medianBlur2', 'gaussianKernel2',
            'medianKernel2', 'OMSE', 'matches', 'acceptance'])
        print df
        # df['gaussianKernel2'] = np.where(df['gaussianBlur2'] == 0, 0,
                # df['gaussianKernel2'])
        # df['medianKernel2'] = np.where(df['medianBlur2'] == 0, 0,
                # df['medianKernel2'])
        temp1= df[['medianKernel2', 'gaussianKernel2', 'matches']].astype(int)
        temp1 = temp1.sort_values(by=['medianKernel2', 'gaussianKernel2'],
                ascending=[True, True])
        print temp1
        temp1 = temp1.pivot(index='gaussianKernel2', columns='medianKernel2',
                values='matches')
        print temp1
        temp2= df[['medianKernel2', 'gaussianKernel2', 'OMSE']].astype(int)
        temp2 = temp2.pivot(index='gaussianKernel2', columns='medianKernel2',
                values='OMSE')
        print temp2

        matplotlib.rc('font', **{'sans-serif' : 'Arial',
                           'family' : 'sans-serif'})
        title = 'SURF-SURF'
        cbartitle1 ='# σωστών αντιστοιχίσεων'
        cbartitle2 = 'Μέσο Τετραγωνικό Σφάλμα Εμποδίων (OMSE)'

        # fig = plt.figure()
        fig, (ax1, ax2) = plt.subplots(1, 2)
        st = fig.suptitle(title, fontsize=20)
        ax1.set_xlabel('Wm')
        ax1.set_ylabel('Wg')
        im1 = ax1.matshow(temp1, interpolation='nearest', cmap=plt.cm.Blues)

        cbar1 = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
        cbar1.set_label(cbartitle1,rotation = 270, labelpad=20)
        tick_spacing = 1
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
        ax1.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

        ax1.set_xticklabels([''] + list(temp1.columns))
        ax1.set_yticklabels([''] + list(temp1.index))

        ax2.set_xlabel('Wm')
        ax2.set_ylabel('Wg')
        im2 = ax2.matshow(temp2, interpolation='nearest', cmap=plt.cm.Blues)

        cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
        cbar2.set_label(cbartitle2,rotation = 270, labelpad=20)
        tick_spacing = 1
        ax2.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
        ax2.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

        ax2.set_xticklabels([''] + list(temp2.columns))
        ax2.set_yticklabels([''] + list(temp2.index))
        fig.tight_layout()
        # shift subplots down:
        st.set_y(0.85)
        fig.subplots_adjust(top=0.85)
        plt.show()
     #    fig = plt.figure()

        # ax = fig.add_subplot(111)
        # ax.set_title(title)
        # ax.set_xlabel('Wm')
        # ax.set_ylabel('Wg')
        # cax = ax.matshow(temp1, interpolation='nearest', cmap=plt.cm.Blues)

        # cbar = fig.colorbar(cax)
        # cbar.set_label('# σωστών αντιστοιχίσεων',rotation = 270, labelpad=
                # 20)
        # tick_spacing = 1
        # ax.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
        # ax.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

        # ax.set_xticklabels([''] + list(temp1.columns))
        # ax.set_yticklabels([''] + list(temp1.index))

        # plt.show()




  
