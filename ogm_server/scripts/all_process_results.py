#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals
import rospy
import rospkg
import sys
import os
import json
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from collections import OrderedDict

if __name__ == "__main__":
    results_path=rospkg.RosPack().get_path("ogm_server")+'/scripts/benchmarking_results/evaluation/map1/ALL/results.json'
    images_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/benchmarking_results/evaluation/map1/ALL/'
    visuals_path = rospkg.RosPack().get_path("ogm_server")+'/scripts/benchmarking_results/evaluation/map1/visual/'
    with open(results_path, 'r') as p:
        json_list = json.load(p, object_pairs_hook=OrderedDict)
        df = pd.DataFrame(json_list, columns= ['#','detector',
            'descriptor','gaussianKernel2', 'medianKernel2',
            'OMSE','quality','meanExTime','overlapAera', 'matches', 'acceptance'])
        df.matches = df.matches.astype(int)
        df.meanExTime = df.meanExTime.astype(float)
        # print df
        df['quality'] = df['quality'].replace([1.00],[0.00])
        df = df[(df['acceptance'] > 0.90) & (df['quality'] > 0.8)]# & (df['OMSE'] > 20)] 
        df = df[(df['meanExTime'] < 7)]
        df1 = df.loc[df.groupby(['detector', 'descriptor'])['quality'].idxmax()]
        # print df1
   #      df2 = df.loc[df.groupby(['gaussianKernel2', 'medianKernel2'])['OMSE'].idxmin()]
        # print df2

 #        temp2 = df1.loc[df1.reset_index().groupby(['detector','descriptor'])['OMSE'].idxmin()]
        # temp1 = df1.loc[df1.reset_index().groupby(['detector','descriptor'])['matches'].idxmax()]
        # temp3 = df1.loc[df1.reset_index().groupby(['detector','descriptor'])['acceptance'].idxmax()]

        temp2 = df1.pivot(index='detector', columns='descriptor',
                values='OMSE')
        temp1 = df1.pivot(index='detector', columns='descriptor',
                values='quality')
        temp3 = df1.pivot(index='detector', columns='descriptor',
                values='matches')
        temp5 = df1.pivot(index='detector', columns='descriptor',
                values='overlapAera')


        temp6 = df1.pivot(index='detector', columns='descriptor',
                values='gaussianKernel2')
        temp7 = df1.pivot(index='detector', columns='descriptor',
                values='medianKernel2')
        temp8 = df.pivot_table(index='detector', columns='descriptor',
                values='meanExTime')

        # temp6 = df2.pivot(index='gaussianKernel2', columns='medianKernel2',
                # values='OMSE')
        # temp7 = df2.pivot(index='gaussianKernel2', columns='medianKernel2',
                # values='matches')

        print temp1.to_latex()
        print temp2.to_latex()
        print temp3.to_latex()
        print temp5.to_latex()
    #     print temp6.to_latex()
        # print temp7.to_latex()
        print temp8.to_latex()
        
        globalMax = np.nanmax(temp3.values)
        minOMSE = np.nanmin(temp2.values)
        print globalMax
        print minOMSE
        best = df1[df1['OMSE'] == minOMSE]
        print best['#']
        best = best.drop(best.columns[[0]], 1)
        print best.to_latex()
        matplotlib.rc('font', **{'sans-serif' : 'Arial',
                           'family' : 'sans-serif'})
        plt.style.use('ggplot')
        fig = plt.figure()
        fig, (ax1, ax2) = plt.subplots(1, 2)
        df.boxplot('meanExTime',['detector'], ax1, fontsize=20)
        ax1.set_ylabel('Χρόνος Εκτέλεσης (sec)')
        plt.xticks(rotation=45)
        df.boxplot('meanExTime',['descriptor'], ax2, fontsize=20)
        st = fig.suptitle("Περιβάλλον 1 - Αξιολόγηση: Διακύμανση Μέσου Χρόνου Εκτέλεσης διαφορετικών συνδυασμών παραμέτρων", fontsize=22)
        ax2.set_ylabel('Χρόνος Εκτέλεσης (sec)')
        plt.xticks(rotation=45)
        fig.autofmt_xdate()
        fig.tight_layout()
        #shift subplots down:
        st.set_y(0.95)
        fig.subplots_adjust(top=0.85)

        fig1=plt.figure()
        fig1, (ax3, ax4) = plt.subplots(1, 2)
        st1 = fig1.suptitle("Περιβάλλον 1 - Αξιολόγηση", fontsize=24)

        df5 = df.groupby(['detector'])['quality'].count().plot(kind='bar',
             ax=ax3, title="Ποσοστό Oρθών Ευθυγραμμίσεων ανά detector",
                legend=False, fontsize=20)
        # manipulate
        vals = ax3.get_yticks()
        ax3.set_yticklabels(['{:3.2f}%'.format(x*100/512) for x in vals])
        df6 = df.groupby(['descriptor'])['quality'].count().plot(kind='bar',
                ax=ax4, title="Ποσοστό Oρθών Ευθυγραμμίσεων ανά descriptor",
                legend=False, fontsize=20)
        vals = ax4.get_yticks()
        ax4.set_yticklabels(['{:3.2f}%'.format(x*100/512) for x in vals])
        plt.xticks(rotation=45)
        fig1.autofmt_xdate()
        fig1.tight_layout()
        #shift subplots down:
        st1.set_y(0.95)
        fig1.subplots_adjust(top=0.85)

        plt.show()
        # df4 = df.groupby(['detector', 'descriptor'])['meanExTime'].mean()

        # print df4
        # print df3['meanExTime'].mean()
        for index, row in df1.iterrows():
            image_path = images_path + "mergedImage" + str(row['#']) + '.png'
            visual_path = visuals_path + "mergedImage" + str(row['#']) + '.png'
            os.rename(image_path, visual_path)
        # print df[(df['detector'] == 'SURF') & (df['descriptor'] == 'CIRCLE INTERSECTIONS') & (df['OMSE'] == 0)]
          # title = 'SURF-SURF'
        # cbartitle1 ='# σωστών αντιστοιχίσεων'
        # cbartitle2 = 'Μέσο Τετραγωνικό Σφάλμα Εμποδίων (OMSE)'

        # # fig = plt.figure()
        # fig, (ax1, ax2) = plt.subplots(1, 2)
        # st = fig.suptitle(title, fontsize=20)
        # ax1.set_xlabel('Wm')
        # ax1.set_ylabel('Wg')
        # im1 = ax1.matshow(temp1, interpolation='nearest', cmap=plt.cm.Blues)

        # cbar1 = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
        # cbar1.set_label(cbartitle1,rotation = 270, labelpad=20)
        # tick_spacing = 1
        # ax1.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
        # ax1.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

        # ax1.set_xticklabels([''] + list(temp1.columns))
        # ax1.set_yticklabels([''] + list(temp1.index))

        # ax2.set_xlabel('Wm')
        # ax2.set_ylabel('Wg')
        # im2 = ax2.matshow(temp2, interpolation='nearest', cmap=plt.cm.Blues)

        # cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
        # cbar2.set_label(cbartitle2,rotation = 270, labelpad=20)
        # tick_spacing = 1
        # ax2.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
        # ax2.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

        # ax2.set_xticklabels([''] + list(temp2.columns))
        # ax2.set_yticklabels([''] + list(temp2.index))
        # fig.tight_layout()
        # # shift subplots down:
        # st.set_y(0.85)
        # fig.subplots_adjust(top=0.85)
        # plt.show()
  


  
