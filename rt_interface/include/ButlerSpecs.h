#ifndef BUTLER_SPECS_H
#define BUTLER_SPECS_H

#define WHEEL_RADIUS					(0.1016)
#define WHEEL_DIA						(WHEEL_RADIUS * 2)
#define ENCODER_TICKS_PER_REV  			(20000)
#define ANGLETOENC(x)					( ( (x) * ENCODER_TICKS_PER_REV ) / ( M_PI * 2 ) )
#define CMTOM							(100.0)
#define WHEEL_BASE						(0.5769)
#define ENCTOLEN(x)						(((x) * (M_PI * WHEEL_DIA)) / ENCODER_TICKS_PER_REV)

#endif  /*BUTLER_SPECS_H*/
