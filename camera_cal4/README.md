# Instrucciones para compilar:
1. En la carpeta camera_cal4 ejecutar $cmake .
2. Ingresar a t1, ejecutar $make
3. ejecutar el código

#Ejecución: 
		./main.out p_distribution p_dir_in p_dir_out p_n_frames p_fronto_par_type p_control_points_type p_min_angle p_max_angle

		p_distribution:
		DIST_RAMDOM|DIST_ANGLE

		p_dir_in:
		ruta del video de entrada

		p_dir_out:
		ruta de carpeta de salida

		p_n_frames:
		numero de frames para calibrar

		p_fronto_par_type
		FP_INS_EXT|FP_PERSPECTIVE

		p_control_points_type
		RP_SIMPLE|RP_COLINEARITY|RP_AVG_SIMPLE_COLINEARITY|RP_BARICENTER

		p_min_angle
		minimo angulo en radianes

		p_max_angle
		max angulo en radianes

#Casos de prueba:
Los casos de prueba ejecutados son:

	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_SIMPLE
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_AVG_SIMPLE_COLINEARITY
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_BARICENTER
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_COLINEARITY

	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER
	./main.out DIST_RAMDOM ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY

	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_SIMPLE 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_AVG_SIMPLE_COLINEARITY 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_BARICENTER 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_PERSPECTIVE RP_COLINEARITY 0 15

	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY 0 15

	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_SIMPLE 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_AVG_SIMPLE_COLINEARITY 0 15
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_BARICENTER 5 65
	./main.out DIST_ANGLE ../res/videos/mkv_ps3.mkv ../res/results/test_dist/ 30 FP_INS_EXT RP_COLINEARITY 5 65
